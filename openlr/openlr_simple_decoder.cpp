#include "openlr/openlr_simple_decoder.hpp"
#include "openlr/openlr_simple_parser.hpp"

#include "routing/car_model.hpp"
#include "routing/car_router.hpp"
#include "routing/features_road_graph.hpp"
#include "routing/road_graph.hpp"
#include "routing/router_delegate.hpp"

#include "indexer/index.hpp"
#include "indexer/scales.hpp"

#include "geometry/angles.hpp"
#include "geometry/latlon.hpp"
#include "geometry/polyline2d.hpp"

#include "platform/location.hpp"

#include "base/logging.hpp"
#include "base/math.hpp"

#include "std/algorithm.hpp"
#include "std/fstream.hpp"
#include "std/queue.hpp"

using namespace routing;

// double constexpr kMwmRoadCrossingRadiusMeters = 2.0;
// m2::RectD const rect = MercatorBounds::RectByCenterXYAndSizeInMeters(cross, kMwmRoadCrossingRadiusMeters);
//   m_index.ForEachInRect(featuresLoader, rect, GetStreetReadScale());
// }

namespace  // A staff to get road data.
{
double constexpr kGradPerPoint = 360.0 / 256.0;

uint32_t BearingToByte(double const angle)
{
  CHECK_LESS_OR_EQUAL(angle, 360, ("Angle should be less than or equal to 360."));
  CHECK_GREATER_OR_EQUAL(angle, 0, ("Angle should be greater than or equal to 0"));
  return angle / kGradPerPoint - 1;
};

double Bearing(m2::PointD const & a, m2::PointD const & b)
{
  return location::AngleToBearing(my::RadToDeg(ang::AngleTo(a, b)));
}

class DijkstraRouter
{
public:
  size_t const kMaxRoadCandidates = 10;
  double const kEps = 1e-9;
  double const kMaxPathLengthM = 6000;

  DijkstraRouter(FeaturesRoadGraph & graph) : m_graph(graph) {}

  bool Go(m2::PointD const & source, m2::PointD const & target, vector<Edge> & path)
  {
    m_graph.ResetFakes();

    Junction const s(source, 0 /* altitude */);
    Junction const t(target, 0 /* altitude */);

    {
      vector<pair<Edge, Junction>> sourceVicinity;
      m_graph.FindClosestEdges(source, kMaxRoadCandidates, sourceVicinity);
      m_graph.AddFakeEdges(s, sourceVicinity);
    }
    {
      vector<pair<Edge, Junction>> targetVicinity;
      m_graph.FindClosestEdges(target, kMaxRoadCandidates, targetVicinity);
      m_graph.AddFakeEdges(t, targetVicinity);
    }

    priority_queue<pair<double, Junction>> queue;
    map<Junction, double> distances;
    map<Junction, Edge> links;

    queue.emplace(0, s);
    distances[s] = 0;

    while (!queue.empty())
    {
      auto const p = queue.top();
      queue.pop();

      double const du = -p.first;
      Junction const u = p.second;

      if (du > distances[u] + kEps)
        continue;

      if (u == t)
      {
        auto cur = t;
        while (!(cur == s))
        {
          auto const & edge = links[cur];
          path.push_back(edge);
          cur = edge.GetStartJunction();
        }
        reverse(path.begin(), path.end());
        return true;
      }

      if (du + Distance(s, t) - Distance(u, t) > kMaxPathLengthM)
        continue;

      IRoadGraph::TEdgeVector edges;
      m_graph.GetOutgoingEdges(u, edges);
      for (auto const & edge : edges)
      {
        auto const & v = edge.GetEndJunction();
        double const dv = du + GetReducedWeight(edge, t);
        if (distances.count(v) == 0 || distances[v] > dv + kEps)
        {
          distances[v] = dv;
          links[v] = edge;
          queue.emplace(-dv, v);
        }
      }
    }

    return false;
  }

private:
  double Distance(Junction const & u, Junction const & v) const
  {
    return MercatorBounds::DistanceOnEarth(u.GetPoint(), v.GetPoint());
  }

  double GetWeight(Edge const & e) const
  {
    double const w = Distance(e.GetStartJunction(), e.GetEndJunction());
    if (e.GetFeatureId().IsValid())
      return w;

    // A penalty for fake edges, actually, there can be any constant
    // greater than one.
    return (1 + kEps) * w;
  }

  double GetReducedWeight(Edge const & e, Junction const & t) const
  {
    double const w = GetWeight(e);
    double const piU = Distance(e.GetStartJunction(), t);
    double const piV = Distance(e.GetEndJunction(), t);
    return max(w + piV - piU, 0.0);
  }

  FeaturesRoadGraph & m_graph;
};

vector<m2::PointD> GetFeaturePoints(FeatureType const & ft)
{
  vector<m2::PointD> points;
  points.reserve(ft.GetPointsCount());
  ft.ForEachPoint([&points](m2::PointD const & p)
                  {
                    points.push_back(p);
                  }, scales::GetUpperScale());
  return points;
}

double GetDistanceToLinearFeature(m2::PointD const & p, FeatureType const & ft)
{
  m2::PolylineD poly(GetFeaturePoints(ft));
  return sqrt(poly. GetShortestSquareDistance(p));
}

vector<FeatureType> GetRoadFeaturesAtPoint(Index const & index,
                                           FeaturesRoadGraph & roadGraph,
                                           ms::LatLon const latLon)
{
  vector<FeatureType> features;

  auto const center = MercatorBounds::FromLatLon(latLon);
  auto const rect = MercatorBounds::RectByCenterXYAndSizeInMeters(center, 25);
  ASSERT_LESS_OR_EQUAL(MercatorBounds::DistanceOnEarth(rect.LeftTop(), rect.LeftBottom()), 50.1, ());
  ASSERT_LESS_OR_EQUAL(MercatorBounds::DistanceOnEarth(rect.RightTop(), rect.RightBottom()), 50.1, ());
  ASSERT_LESS_OR_EQUAL(MercatorBounds::DistanceOnEarth(rect.LeftTop(), rect.RightTop()), 50.1, ());
  ASSERT_LESS_OR_EQUAL(MercatorBounds::DistanceOnEarth(rect.LeftBottom(), rect.RightBottom()), 50.1, ());

  index.ForEachInRect([&features, &roadGraph, &center](FeatureType const & ft) {
      ft.ParseHeader2();
      if (!roadGraph.IsRoad(ft))
        return;

      // TODO(mgsergio): Parse only necessary fields.
      ft.ParseEverything();

      if (ft.GetPointsCount() == 1)
      {
        LOG(LDEBUG, ("A linear feature with one"));
        return;
      }

      auto constexpr kMaxDistanceMeters = 10.0;
      // if (GetDistanceToLinearFeature(center, ft) >= kMaxDistanceMeters)
      //   return;

      auto good = false;
      for (auto const & p : GetFeaturePoints(ft))
      {
        if (MercatorBounds::DistanceOnEarth(p, center) <= kMaxDistanceMeters)
        {
          good = true;
          break;
        }
      }

      if (!good)
        return;

      features.push_back(ft);
    },
    rect, scales::GetUpperScale());

  return features;
}

IRoadGraph::TEdgeVector GetOutgoingEdges(FeaturesRoadGraph const & roadGraph,
                                         ms::LatLon const & latLon)
{
  IRoadGraph::TEdgeVector edges;

  Junction junction(MercatorBounds::FromLatLon(latLon), feature::kDefaultAltitudeMeters);
  roadGraph.GetOutgoingEdges(junction, edges);

  return edges;
}

IRoadGraph::TEdgeVector GetIngoingEdges(FeaturesRoadGraph const & roadGraph,
                                        ms::LatLon const & latLon)
{
  IRoadGraph::TEdgeVector edges;

  Junction junction(MercatorBounds::FromLatLon(latLon), feature::kDefaultAltitudeMeters);
  roadGraph.GetIngoingEdges(junction, edges);

  return edges;
}

bool CalculateRoute(IRouter & router, ms::LatLon const & first,
                    ms::LatLon const & last, vector<m2::PointD> & points)
{
  Route route("olr-route");

  RouterDelegate delegate;
  auto const result =
      router.CalculateRoute(MercatorBounds::FromLatLon(first), {0.0, 0.0} /* direction */,
                            MercatorBounds::FromLatLon(last), delegate, route);
  if (result != IRouter::ResultCode::NoError)
  {
    LOG(LDEBUG, ("Can't calculate route for points", first, last, ", code:", result));
    return false;
  }

  points = route.GetPoly().GetPoints();
  return true;
}

void LeaveEdgesStartedFrom(Junction const & junction, IRoadGraph::TEdgeVector & edges)
{
  auto count = 0;
  auto const it = remove_if(begin(edges), end(edges), [&junction, &count](Edge const & e)
  {
    bool const eq = !AlmostEqualAbs(e.GetStartJunction(), junction);
    LOG(LDEBUG, (e.GetStartJunction().GetPoint(), "!=", junction.GetPoint(), "->", eq));
    count += eq;
    return eq;
  });
  LOG(LDEBUG, (count, "values should be removed from vector of legth", edges.size()));

  if (it != end(edges))
    edges.erase(it, end(edges));

  LOG(LDEBUG, ("edges current size", edges.size()));
}

Junction JunctionFromPoint(m2::PointD const & p)
{
  return {p, feature::kDefaultAltitudeMeters};
}

struct Stats
{
  uint32_t m_shortRoutes = 0;
  uint32_t m_zeroCanditates = 0;
  uint32_t m_moreThanOneCandidates = 0;
  uint32_t m_routeIsNotCalculated = 0;
  uint32_t m_total = 0;
};

routing::IRoadGraph::TEdgeVector ReconstructPath(routing::IRoadGraph const & graph,
                                                 routing::Route & route, Stats & stats)
{
  routing::IRoadGraph::TEdgeVector path;

  auto poly = route.GetPoly().GetPoints();
  // There are zero-length linear features, so poly can contain adjucent duplications.
  poly.erase(unique(begin(poly), end(poly), [](m2::PointD const & a, m2::PointD const & b)
  {
    return my::AlmostEqualAbs(a, b, routing::kPointsEqualEpsilon);
  }), end(poly));

  // TODO(mgsergio): A rote may strart/end at poits other than edge ends, this situation
  // shoud be handled separately.
  if (poly.size() < 4)
  {
    ++stats.m_shortRoutes;
    LOG(LINFO, ("Short polylines are not handled yet."));
    return {};
  }

  routing::IRoadGraph::TEdgeVector edges;
  // Start from the second point of the route that is the start of the egge for shure.
  auto it = next(begin(poly));
  auto prevJunction = JunctionFromPoint(*it++);

  // Stop at the last point that is the end of the edge for shure.
  for (; it != prev(end(poly)); prevJunction = JunctionFromPoint(*it++))
  {
    // TODO(mgsergio): Check edges are not fake;
    graph.GetIngoingEdges(JunctionFromPoint(*it), edges);
    LOG(LDEBUG, ("Edges extracted:", edges.size()));
    LeaveEdgesStartedFrom(prevJunction, edges);
    if (edges.size() > 1)
    {
      ++stats.m_moreThanOneCandidates;
      LOG(LDEBUG, ("More than one edge candidate."));
    }
    else if (edges.size() == 0)
    {
      ++stats.m_zeroCanditates;
      LOG(LDEBUG, ("Zero edge candidate extracted."));
      // Sometimes a feature may be duplicated in two or more mwms: MAPSME-2816.
      // ASSERT_GREATER_OR_EQUAL(edges.size(), 1,
      //                         ("There should be at least one adge. (One in normal case)"));
      return {};
    }

    auto const & edge = edges.front();
    if (!edge.GetFeatureId().m_mwmId.IsAlive())
      continue;

    path.push_back(edges.front());
    edges.clear();
  }

  // TODO(mgsergio): A rote may strart/end at poits other than edge ends, this situation
  // shoud be handled separately.
  if (edges.size() < 3)
  {
    ++stats.m_shortRoutes;
    LOG(LINFO, ("Short polylines are not handled yet."));
    return {};
  }

  return path;
}
}  // namespace

namespace openlr
{
int const OpenLRSimpleDecoder::kHandleAllSegmets = -1;

OpenLRSimpleDecoder::OpenLRSimpleDecoder(string const & dataFilename, Index const & index,
                                         IRouter & router)
  : m_index(index), m_router(router)
{
  auto const load_result = m_document.load_file(dataFilename.data());
  if (!load_result)
    MYTHROW(DecoderError, ("Can't load file", dataFilename, ":", load_result.description()));
}

void OpenLRSimpleDecoder::Decode(int const segmentsTohandle)
{
  FeaturesRoadGraph roadGraph(m_index, IRoadGraph::Mode::ObeyOnewayTag,
                              make_unique<CarModelFactory>());
  DijkstraRouter router(roadGraph);
  Stats stats;

  ofstream sample("inrix_vs_mwm.txt");

  // TODO(mgsergio): Feed segments derectly to the decoder. Parsing sholud not
  // take place inside decoder process.
  vector<LinearSegment> segments;
  if (!ParseOpenlr(m_document, segments))
    MYTHROW(DecoderError, ("Can't parse data."));

  for (auto const & segment : segments)
  {
    if (stats.m_total != 0)
      LOG(LINFO, ("Processed:", stats.m_total));
    if (segmentsTohandle != kHandleAllSegmets && stats.m_total == segmentsTohandle)
      break;

    ++stats.m_total;

    // auto const outgoingEdges = GetOutgoingEdges(roadGraph, first);
    // auto const ingoingEdges = GetIngoingEdges(roadGraph, last);

    // LOG(LINFO, ("Number of outgoing enges:", outgoingEdges.size(), "for point: ", first));
    // LOG(LINFO, ("Number of ingoing enges:", ingoingEdges.size(), "for point: ", last));

    // auto const featuresForFirst = GetRoadFeaturesAtPoint(m_index, roadGraph, first);
    // auto const featuresForLast = GetRoadFeaturesAtPoint(m_index, roadGraph, last);

    // cout << "Features: " << featuresForFirst.size() << "\tfor point" << DebugPrint(first) << endl;
    // cout << "Features: " << featuresForLast.size() << "\tfor point" << DebugPrint(last) << endl;

    // LOG(LINFO, ("Number of features", featuresForFirst.size(), "for first point: ", first));
    // LOG(LINFO, ("Number of features", featuresForLast.size(), "for last point: ", last));

    auto const firstPoint = segment.m_locationReference.m_points.front().m_latLon;
    auto const lastPoint = segment.m_locationReference.m_points.back().m_latLon;
    auto const segmentID = segment.m_segmentId;
    LOG(LDEBUG, ("Calculating route from ", firstPoint, lastPoint, "for segment:", segmentID));

    IRoadGraph::TEdgeVector path;
    if (!router.Go(MercatorBounds::FromLatLon(firstPoint), MercatorBounds::FromLatLon(lastPoint),
                   path))
    {
      ++stats.m_routeIsNotCalculated;
      continue;
    }

    path.erase(remove_if(path.begin(), path.end(),
                         [](Edge const & edge) { return !edge.GetFeatureId().IsValid(); }),
               path.end());

    if (path.size() < 4)
      continue;

    sample << segmentID << '\t';
    for (auto it = begin(path); it != end(path); ++it)
    {
      auto const & fid = it->GetFeatureId();
      if (!fid.IsValid())
        continue;
      sample << fid.m_mwmId.GetInfo()->GetCountryName() << '-'
             << fid.m_index << '-' << it->GetSegId();
      if (next(it) != end(path))
        sample << '=';
    }
    sample << endl;
  }

  LOG(LINFO, ("Parsed inrix regments:", stats.m_total,
              "Routes failed:", stats.m_routeIsNotCalculated,
              "Short routes:", stats.m_shortRoutes,
              "Ambiguous routes:", stats.m_moreThanOneCandidates,
              "Path is not reconstructed:", stats.m_zeroCanditates));
}
}  // namespace openlr

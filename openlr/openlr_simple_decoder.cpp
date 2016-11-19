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
#include "std/utility.hpp"

using namespace std::rel_ops;
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

struct InrixPoint
{
  InrixPoint(): m_point(0, 0), m_distanceToNextPointM(0.0) {}

  InrixPoint(openlr::LocationReferencePoint const & lrp)
    : m_point(MercatorBounds::FromLatLon(lrp.m_latLon))
    , m_distanceToNextPointM(lrp.m_distanceToNextPoint)
  {
  }

  m2::PointD m_point;
  double m_distanceToNextPointM;
};

size_t const kMaxRoadCandidates = 10;
double const kDistanceAccuracyM = 1000;
double const kEps = 1e-9;

class AStarRouter
{
public:
  AStarRouter(FeaturesRoadGraph & graph) : m_graph(graph) {}

  bool Go(vector<InrixPoint> const & points, vector<Edge> & path)
  {
    CHECK_GREATER_OR_EQUAL(points.size(), 2, ());

    m_graph.ResetFakes();

    m_pivots.clear();
    for (size_t i = 1; i + 1 < points.size(); ++i)
    {
      m_pivots.emplace_back();
      auto & ps = m_pivots.back();

      vector<pair<Edge, Junction>> vicinity;
      m_graph.FindClosestEdges(points[i].m_point, kMaxRoadCandidates, vicinity);
      for (auto const & v : vicinity)
      {
        auto const & e = v.first;
        ps.push_back(e.GetStartJunction().GetPoint());
        ps.push_back(e.GetEndJunction().GetPoint());
      }
    }
    m_pivots.push_back({points.back().m_point});
    CHECK_EQUAL(m_pivots.size() + 1, points.size(), ());

    m_bounds.resize(points.size() - 1);
    m_bounds[0] = points[0].m_distanceToNextPointM;
    for (size_t i = 1; i < m_bounds.size(); ++i)
      m_bounds[i] = m_bounds[i - 1] + points[i].m_distanceToNextPointM;

    Vertex const s(Junction(points.front().m_point, 0 /* altitude */), 0 /* stage */);
    Vertex const t(Junction(points.back().m_point, 0 /* altitude */),
                   m_pivots.size() - 1 /* stage */);

    {
      vector<pair<Edge, Junction>> sourceVicinity;
      m_graph.FindClosestEdges(points.front().m_point, kMaxRoadCandidates, sourceVicinity);
      m_graph.AddFakeEdges(s.m_junction, sourceVicinity);
    }
    {
      vector<pair<Edge, Junction>> targetVicinity;
      m_graph.FindClosestEdges(points.back().m_point, kMaxRoadCandidates, targetVicinity);
      m_graph.AddFakeEdges(t.m_junction, targetVicinity);
    }

    using State = pair<Score, Vertex>;
    priority_queue<State, vector<State>, greater<State>> queue;
    map<Vertex, Score> scores;
    map<Vertex, pair<Vertex, Edge>> links;

    scores[s] = Score();
    queue.emplace(scores[s], s);

    double const piS = GetPotential(s);

    while (!queue.empty())
    {
      auto const p = queue.top();
      queue.pop();

      Score const & su = p.first;
      Vertex const & u = p.second;
      size_t const stage = u.m_stage;

      if (su != scores[u])
        continue;

      double const piU = GetPotential(u);

      if (u == t)
      {
        auto cur = t;
        while (cur != s)
        {
          auto const & p = links[cur];
          path.push_back(p.second);
          cur = p.first;
        }
        reverse(path.begin(), path.end());
        return true;
      }

      double const ud = su.GetDistance() + piS - piU;  // real distance to u

      // max(kDistanceAccuracyM, m_distanceToNextPointM) is added here
      // to throw out quite long paths.
      if (ud > m_bounds[stage] + max(kDistanceAccuracyM, points[stage].m_distanceToNextPointM))
        continue;

      if (piU < kEps && stage + 1 < m_pivots.size())
      {
        Vertex uu(u.m_junction, u.m_stage + 1);

        double const piUU = GetPotential(uu);

        Score suu = su;
        suu.AddDistance(max(piUU - piU, 0.0));
        suu.AddIntermediateErrorPenalty(
            MercatorBounds::DistanceOnEarth(u.m_junction.GetPoint(), points[stage + 1].m_point));

        if (scores.count(uu) == 0 || scores[uu] > suu)
        {
          scores[uu] = suu;
          links[uu] = make_pair(u, Edge());
          queue.emplace(suu, uu);
        }
      }

      IRoadGraph::TEdgeVector edges;
      m_graph.GetOutgoingEdges(u.m_junction, edges);
      for (auto const & edge : edges)
      {
        Vertex v(edge.GetEndJunction(), stage);
        double const piV = GetPotential(v);

        Score sv = su;
        double const w = GetWeight(edge);
        sv.AddDistance(max(GetWeight(edge) + piV - piU, 0.0));

        double const vd = ud + w;  // real distance to v
        if (vd > m_bounds[stage])
          sv.AddDistanceErrorPenalty(std::min(vd - m_bounds[stage], w));

        if (!edge.GetFeatureId().IsValid())
          sv.AddFakePenalty(w);

        if (scores.count(v) == 0 || scores[v] > sv)
        {
          scores[v] = sv;
          links[v] = make_pair(u, edge);
          queue.emplace(sv, v);
        }
      }
    }

    return false;
  }

private:
  struct Vertex
  {
    Vertex() = default;
    Vertex(Junction const & junction, size_t stage) : m_junction(junction), m_stage(stage) {}

    inline bool operator<(Vertex const & rhs) const
    {
      if (m_stage != rhs.m_stage)
        return m_stage < rhs.m_stage;
      return m_junction < rhs.m_junction;
    }

    inline bool operator==(Vertex const & rhs) const
    {
      return m_junction == rhs.m_junction && m_stage == rhs.m_stage;
    }

    Junction m_junction;
    size_t m_stage = 0;
  };

  class Score
  {
  public:
    inline double GetDistance() const { return m_distance; }

    inline void AddDistance(double p)
    {
      m_distance += p;
      Update();
    }

    inline void AddFakePenalty(double p)
    {
      m_fakePenalty += p;
      Update();
    }

    inline void AddIntermediateErrorPenalty(double p)
    {
      m_intermediateErrorPenalty += p;
      Update();
    }

    inline void AddDistanceErrorPenalty(double p)
    {
      m_distanceErrorPenalty += p;
      Update();
    }

    bool operator<(Score const & rhs) const { return m_score < rhs.m_score; }
    bool operator==(Score const & rhs) const { return m_score == rhs.m_score; }

  private:
    void Update()
    {
      m_score =
          m_distance + 2 * (m_fakePenalty + m_intermediateErrorPenalty + m_distanceErrorPenalty);
    }

    // Reduced length of path in meters.
    double m_distance = 0.0;

    // Penalty of passing by fake edges.
    double m_fakePenalty = 0.0;

    // Penalty of passing through an candidate that is too far from
    // corresponding intermediate point.
    double m_intermediateErrorPenalty = 0.0;

    // Penalty of path between consecutive points that is longer than
    // required.
    double m_distanceErrorPenalty = 0.0;

    // Total score.
    double m_score = 0.0;
  };

  double GetPotential(Vertex const & u) const
  {
    CHECK_LESS(u.m_stage, m_pivots.size(), ());

    auto const & pivots = m_pivots[u.m_stage];

    double potential = numeric_limits<double>::max();

    auto const & point = u.m_junction.GetPoint();
    for (auto const & pivot : pivots)
      potential = min(potential, MercatorBounds::DistanceOnEarth(pivot, point));
    return potential;
  }

  double Distance(Junction const & u, Junction const & v) const
  {
    return MercatorBounds::DistanceOnEarth(u.GetPoint(), v.GetPoint());
  }

  double GetWeight(Edge const & e) const
  {
    return Distance(e.GetStartJunction(), e.GetEndJunction());
  }

  FeaturesRoadGraph & m_graph;
  vector<vector<m2::PointD>> m_pivots;
  vector<double> m_bounds;
  m2::PointD m_target;
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

void OpenLRSimpleDecoder::Decode(int const segmentsTohandle, bool const multipointsOnly)
{
  FeaturesRoadGraph roadGraph(m_index, IRoadGraph::Mode::ObeyOnewayTag,
                              make_unique<CarModelFactory>());
  AStarRouter astarRouter(roadGraph);
  Stats stats;

  ofstream sample("inrix_vs_mwm.txt");

  // TODO(mgsergio): Feed segments derectly to the decoder. Parsing sholud not
  // take place inside decoder process.
  vector<LinearSegment> segments;
  if (!ParseOpenlr(m_document, segments))
    MYTHROW(DecoderError, ("Can't parse data."));

  for (auto const & segment : segments)
  {
    auto const & ref = segment.m_locationReference;

    if (ref.m_points.size() == 2 && multipointsOnly)
      continue;

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

    vector<InrixPoint> points;
    for (auto const & point : ref.m_points)
      points.emplace_back(point);

    IRoadGraph::TEdgeVector path;
    if (!astarRouter.Go(points, path))
    {
      ++stats.m_routeIsNotCalculated;
      continue;
    }

    path.erase(remove_if(path.begin(), path.end(),
                         [](Edge const & edge) { return !edge.GetFeatureId().IsValid(); }),
               path.end());

    if (path.size() < 4)
    {
      LOG(LINFO, ("Skipping short path."));
      continue;
    }

    sample << segment.m_segmentId << '\t';
    for (auto it = begin(path); it != end(path); ++it)
    {
      auto const & fid = it->GetFeatureId();
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

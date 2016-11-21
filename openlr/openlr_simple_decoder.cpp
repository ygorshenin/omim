#include "openlr/openlr_simple_decoder.hpp"
#include "openlr/openlr_model.hpp"
#include "openlr/openlr_simple_parser.hpp"

#include "routing/car_model.hpp"
#include "routing/car_router.hpp"
#include "routing/features_road_graph.hpp"
#include "routing/road_graph.hpp"
#include "routing/router_delegate.hpp"

#include "indexer/classificator.hpp"
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
#include "std/map.hpp"
#include "std/queue.hpp"
#include "std/utility.hpp"

using namespace std::rel_ops;
using namespace routing;

namespace  // A staff to get road data.
{
size_t const kMaxRoadCandidates = 10;
double const kDistanceAccuracyM = 1000;
double const kEps = 1e-9;
double const kBearingDist = 25;
double const kAnglesInBucket = 360.0 / 256;

uint32_t BearingToByte(double const angle)
{
  CHECK_LESS_OR_EQUAL(angle, 360, ("Angle should be less than or equal to 360."));
  CHECK_GREATER_OR_EQUAL(angle, 0, ("Angle should be greater than or equal to 0"));
  return my::clamp(angle / kAnglesInBucket, 0, 255);
};

double Bearing(m2::PointD const & a, m2::PointD const & b)
{
  return location::AngleToBearing(my::RadToDeg(ang::AngleTo(a, b)));
}

class RoadInfoGetter
{
public:
  struct RoadInfo
  {
    openlr::FunctionalRoadClass m_frc;
    openlr::FormOfAWay m_fow;
  };

  RoadInfoGetter(Index const & index): m_index(index) {}

  RoadInfo GetFeatureRoadInfo(FeatureID const & fid)
  {
    auto it = m_cache.find(fid);
    if (it == end(m_cache))
    {
      Index::FeaturesLoaderGuard g(m_index, fid.m_mwmId);
      FeatureType ft;
      CHECK(g.GetOriginalFeatureByIndex(fid.m_index, ft), ());
      ft.ParseTypes();
      RoadInfo info;
      info.m_frc = GetFunctionalRoadClass(ft);
      info.m_fow = GetFormOfAWay(ft);
      it = m_cache.emplace(fid, info).first;
    }

    return it->second;
  }

private:
  static openlr::FunctionalRoadClass GetFunctionalRoadClass(feature::TypesHolder const & types)
  {
    auto const & c = classif();

    if (types.Has(c.GetTypeByPath({"highway", "motorway"})) ||
        types.Has(c.GetTypeByPath({"highway", "trunk"})))
    {
      return openlr::FunctionalRoadClass::FRC0;
    }

    else if (types.Has(c.GetTypeByPath({"highway", "primary"})) ||
             types.Has(c.GetTypeByPath({"highway", "primary_link"})))
    {
      return openlr::FunctionalRoadClass::FRC1;
    }

    else if (types.Has(c.GetTypeByPath({"highway", "secondary"})) ||
             types.Has(c.GetTypeByPath({"highway", "secondary_link"})))
    {
      return openlr::FunctionalRoadClass::FRC2;
    }

    else if (types.Has(c.GetTypeByPath({"highway", "tertiary"})) ||
             types.Has(c.GetTypeByPath({"highway", "tertiary_link"})))
    {
      return openlr::FunctionalRoadClass::FRC3;
    }

    else if (types.Has(c.GetTypeByPath({"highway", "road"})) ||
             // types.Has(c.GetTypeByPath({"highway", "road_link"})) ||
             types.Has(c.GetTypeByPath({"highway", "unclassified"})) ||
             types.Has(c.GetTypeByPath({"highway", "residential"})))
    {
      return openlr::FunctionalRoadClass::FRC4;
    }

    else if (types.Has(c.GetTypeByPath({"highway", "living_street"})))
    {
      return openlr::FunctionalRoadClass::FRC5;
    }

    return openlr::FunctionalRoadClass::FRC7;
  }

  static openlr::FormOfAWay GetFormOfAWay(feature::TypesHolder const & types)
  {
    auto const & c = classif();

    if (types.Has(c.GetTypeByPath({"highway", "motorway"})) ||
        types.Has(c.GetTypeByPath({"highway", "trunk"})))
    {
      return openlr::FormOfAWay::Motorway;
    }

    else if (types.Has(c.GetTypeByPath({"highway", "primary"})) ||
             types.Has(c.GetTypeByPath({"highway", "primary_link"})))
    {
      return openlr::FormOfAWay::MultipleCarriageway;
    }

    return openlr::FormOfAWay::SingleCarriageway;
  }

  Index const & m_index;
  map<FeatureID, RoadInfo> m_cache;
};

struct InrixPoint
{
  InrixPoint() = default;

  InrixPoint(openlr::LocationReferencePoint const & lrp)
    : m_point(MercatorBounds::FromLatLon(lrp.m_latLon))
    , m_distanceToNextPointM(lrp.m_distanceToNextPoint)
    , m_bearing(lrp.m_bearing)
    , m_lfrcnp(lrp.m_functionalRoadClass)
  {
  }

  m2::PointD m_point = m2::PointD::Zero();
  double m_distanceToNextPointM = 0.0;
  uint8_t m_bearing = 0;
  openlr::FunctionalRoadClass m_lfrcnp = openlr::FunctionalRoadClass::NotAValue;
};

class AStarRouter
{
public:
  AStarRouter(FeaturesRoadGraph & graph, RoadInfoGetter & roadInfoGetter)
    : m_graph(graph)
    , m_roadInfoGetter(roadInfoGetter)
  {
  }

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

    Junction js(points.front().m_point, 0 /* altitude */);

    {
      vector<pair<Edge, Junction>> sourceVicinity;
      m_graph.FindClosestEdges(points.front().m_point, kMaxRoadCandidates, sourceVicinity);
      m_graph.AddFakeEdges(js, sourceVicinity);
    }

    Junction jt(points.back().m_point, 0 /* altitude */);
    {
      vector<pair<Edge, Junction>> targetVicinity;
      m_graph.FindClosestEdges(points.back().m_point, kMaxRoadCandidates, targetVicinity);
      m_graph.AddFakeEdges(jt, targetVicinity);
    }

    using State = pair<Score, Vertex>;
    priority_queue<State, vector<State>, greater<State>> queue;
    map<Vertex, Score> scores;
    Links links;

    Vertex const s(js, js, 0, 0 /* stage */);
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

      if (u.m_stage == m_pivots.size())
      {
        auto cur = u;
        while (cur != s)
        {
          auto const & p = links[cur];
          path.push_back(p.second);
          cur = p.first;
        }
        reverse(path.begin(), path.end());
        return true;
      }

      if (u.m_junction == jt && u.m_stage + 1 == m_pivots.size())
      {
        Vertex v(jt, jt, 0, u.m_stage + 1);
        Score sv = su;

        int const expected = points.back().m_bearing;
        int const actual = GetReverseBearing(u, links);
        sv.AddBearingPenalty(expected, actual);

        if ((scores.count(v) == 0 || scores[v] > sv) && u != v)
        {
          scores[v] = sv;
          links[v] = make_pair(u, Edge::MakeFake(u.m_junction, v.m_junction));
          queue.emplace(sv, v);
        }

        continue;
      }

      double const piU = GetPotential(u);
      double const ud = su.GetDistance() + piS - piU;  // real distance to u

      // max(kDistanceAccuracyM, m_distanceToNextPointM) is added here
      // to throw out quite long paths.
      if (ud > m_bounds[stage] + max(kDistanceAccuracyM, points[stage].m_distanceToNextPointM))
        continue;

      if (piU < kEps && stage + 1 < m_pivots.size())
      {
        Vertex uu(u.m_junction, u.m_junction, ud, u.m_stage + 1);

        double const piUU = GetPotential(uu);

        Score suu = su;
        suu.AddDistance(max(piUU - piU, 0.0));
        suu.AddIntermediateErrorPenalty(
            MercatorBounds::DistanceOnEarth(u.m_junction.GetPoint(), points[stage + 1].m_point));

        if (ud - u.m_stageStartDistance < kBearingDist && u.m_junction != u.m_stageStart)
        {
          int const expected = points[u.m_stage].m_bearing;
          int const actual =
              BearingToByte(Bearing(u.m_stageStart.GetPoint(), u.m_junction.GetPoint()));
          suu.AddBearingPenalty(expected, actual);
        }

        if ((scores.count(uu) == 0 || scores[uu] > suu) && u != uu)
        {
          scores[uu] = suu;
          links[uu] = make_pair(u, Edge::MakeFake(u.m_junction, uu.m_junction));
          queue.emplace(suu, uu);
        }
      }

      IRoadGraph::TEdgeVector edges;
      m_graph.GetOutgoingEdges(u.m_junction, edges);
      for (auto const & edge : edges)
      {
        {
          if (stage + 1 != points.size() && !edge.IsFake())
          {
            auto const lowestFuncRoadClass = points[stage].m_lfrcnp;
            auto const edgeFuncRoadClass =
                m_roadInfoGetter.GetFeatureRoadInfo(edge.GetFeatureId()).m_frc;
            if (edgeFuncRoadClass < lowestFuncRoadClass)
              continue;
          }
        }

        Vertex v(edge.GetEndJunction(), u.m_stageStart, u.m_stageStartDistance, stage);
        double const piV = GetPotential(v);

        Score sv = su;
        double const w = GetWeight(edge);
        sv.AddDistance(max(GetWeight(edge) + piV - piU, 0.0));

        double const vd = ud + w;  // real distance to v
        if (ud < u.m_stageStartDistance + kBearingDist &&
            vd >= u.m_stageStartDistance + kBearingDist)
        {
          double const delta = vd - u.m_stageStartDistance - kBearingDist;
          auto const p = PointAtSegment(edge.GetStartJunction().GetPoint(),
                                        edge.GetEndJunction().GetPoint(), delta);
          int const expected = points[u.m_stage].m_bearing;
          int const actual = BearingToByte(Bearing(u.m_stageStart.GetPoint(), p));
          sv.AddBearingPenalty(expected, actual);
        }

        if (vd > m_bounds[stage])
          sv.AddDistanceErrorPenalty(std::min(vd - m_bounds[stage], w));

        if (edge.IsFake())
          sv.AddFakePenalty(w);

        if ((scores.count(v) == 0 || scores[v] > sv) && v != u)
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
    Vertex(Junction const & junction, Junction const & stageStart, double stageStartDistance,
           size_t stage)
      : m_junction(junction)
      , m_stageStart(stageStart)
      , m_stageStartDistance(stageStartDistance)
      , m_stage(stage)
    {
    }

    inline bool operator<(Vertex const & rhs) const
    {
      if (m_stage != rhs.m_stage)
        return m_stage < rhs.m_stage;
      if (m_junction != rhs.m_junction)
        return m_junction < rhs.m_junction;
      return m_stageStart < rhs.m_stageStart;
    }

    inline bool operator==(Vertex const & rhs) const
    {
      return m_junction == rhs.m_junction && m_stageStart == rhs.m_stageStart &&
             m_stage == rhs.m_stage;
    }

    Junction m_junction;
    Junction m_stageStart;
    double m_stageStartDistance = 0.0;
    size_t m_stage = 0;
  };

  using Links = map<Vertex, pair<Vertex, Edge>>;

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

    inline void AddBearingPenalty(int expected, int actual)
    {
      double angle = my::DegToRad(abs(expected - actual) * kAnglesInBucket);
      m_bearingPenalty += angle * kBearingDist;
    }

    bool operator<(Score const & rhs) const { return m_score < rhs.m_score; }
    bool operator==(Score const & rhs) const { return m_score == rhs.m_score; }

  private:
    void Update()
    {
      m_score = m_distance +
                3 * (m_fakePenalty + m_intermediateErrorPenalty + m_distanceErrorPenalty +
                     m_bearingPenalty);
      CHECK_GREATER_OR_EQUAL(m_score, 0, ());
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

    double m_bearingPenalty = 0.0;

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

  uint32_t GetReverseBearing(Vertex const & u, Links const & links) const
  {
    m2::PointD const a = u.m_junction.GetPoint();
    m2::PointD b = m2::PointD::Zero();

    Vertex curr = u;
    double passed = 0;
    bool found = false;
    while (true)
    {
      auto const it = links.find(curr);
      if (it == links.end())
        break;

      auto const & p = it->second;
      auto const & prev = p.first;
      auto const & edge = p.second;

      if (prev.m_stage != curr.m_stage)
        break;

      double const weight = GetWeight(edge);

      if (passed + weight >= kBearingDist)
      {
        double const delta = kBearingDist - passed;
        b = PointAtSegment(edge.GetEndJunction().GetPoint(), edge.GetStartJunction().GetPoint(),
                           delta);
        found = true;
        break;
      }

      passed += weight;
      curr = prev;
    }
    if (!found)
      b = curr.m_junction.GetPoint();
    return BearingToByte(Bearing(a, b));
  }

  FeaturesRoadGraph & m_graph;
  RoadInfoGetter & m_roadInfoGetter;
  vector<vector<m2::PointD>> m_pivots;
  vector<double> m_bounds;
  m2::PointD m_target;
};

struct Stats
{
  uint32_t m_shortRoutes = 0;
  uint32_t m_zeroCanditates = 0;
  uint32_t m_moreThanOneCandidates = 0;
  uint32_t m_routeIsNotCalculated = 0;
  uint32_t m_total = 0;
};
}  // namespace

namespace openlr
{
int const OpenLRSimpleDecoder::kHandleAllSegmets = -1;

OpenLRSimpleDecoder::OpenLRSimpleDecoder(string const & dataFilename, Index const & index)
  : m_index(index)
{
  auto const load_result = m_document.load_file(dataFilename.data());
  if (!load_result)
    MYTHROW(DecoderError, ("Can't load file", dataFilename, ":", load_result.description()));
}

void OpenLRSimpleDecoder::Decode(int const segmentsTohandle, bool const multipointsOnly)
{
  FeaturesRoadGraph roadGraph(m_index, IRoadGraph::Mode::ObeyOnewayTag,
                              make_unique<CarModelFactory>());
  RoadInfoGetter roadInfoGetter(m_index);
  AStarRouter astarRouter(roadGraph, roadInfoGetter);
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

    vector<InrixPoint> points;
    for (auto const & point : ref.m_points)
      points.emplace_back(point);

    IRoadGraph::TEdgeVector path;
    if (!astarRouter.Go(points, path))
    {
      ++stats.m_routeIsNotCalculated;
      continue;
    }

    path.erase(remove_if(path.begin(), path.end(), [](Edge const & edge) { return edge.IsFake(); }),
               path.end());

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

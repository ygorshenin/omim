#include "openlr/openlr_simple_decoder.hpp"
#include "openlr/openlr_model.hpp"
#include "openlr/openlr_simple_parser.hpp"

#include "routing/car_model.hpp"
#include "routing/car_router.hpp"
#include "routing/features_road_graph.hpp"
#include "routing/road_graph.hpp"
#include "routing/router_delegate.hpp"

#include "indexer/classificator.hpp"
#include "indexer/ftypes_matcher.hpp"
#include "indexer/index.hpp"
#include "indexer/scales.hpp"

#include "geometry/angles.hpp"
#include "geometry/latlon.hpp"
#include "geometry/polyline2d.hpp"
#include "geometry/segment2d.hpp"

#include "platform/location.hpp"

#include "base/logging.hpp"
#include "base/math.hpp"
#include "base/stl_helpers.hpp"

#include "std/algorithm.hpp"
#include "std/fstream.hpp"
#include "std/functional.hpp"
#include "std/map.hpp"
#include "std/queue.hpp"
#include "std/thread.hpp"
#include "std/transform_iterator.hpp"
#include "std/utility.hpp"

using namespace routing;

namespace
{
size_t constexpr kCacheLineSize = 64;

size_t const kMaxRoadCandidates = 10;
double const kDistanceAccuracyM = 1000;
double const kEps = 1e-9;
double const kBearingDist = 25;
double const kAnglesInBucket = 360.0 / 256;

size_t constexpr GCD(size_t a, size_t b) { return b == 0 ? a : GCD(b, a % b); }

size_t constexpr LCM(size_t a, size_t b) { return a / GCD(a, b) * b; }

uint32_t Bearing(m2::PointD const & a, m2::PointD const & b)
{
  auto const angle = location::AngleToBearing(my::RadToDeg(ang::AngleTo(a, b)));
  CHECK_LESS_OR_EQUAL(angle, 360, ("Angle should be less than or equal to 360."));
  CHECK_GREATER_OR_EQUAL(angle, 0, ("Angle should be greater than or equal to 0"));
  return my::clamp(angle / kAnglesInBucket, 0, 255);
}

class TrunkChecker : public ftypes::BaseChecker
{
public:
  TrunkChecker()
  {
    auto const & c = classif();
    m_types.push_back(c.GetTypeByPath({"highway", "motorway"}));
    m_types.push_back(c.GetTypeByPath({"highway", "motorway_link"}));
    m_types.push_back(c.GetTypeByPath({"highway", "trunk"}));
    m_types.push_back(c.GetTypeByPath({"highway", "trunk_link"}));
  }
};

class PrimaryChecker : public ftypes::BaseChecker
{
public:
  PrimaryChecker()
  {
    auto const & c = classif();
    m_types.push_back(c.GetTypeByPath({"highway", "primary"}));
    m_types.push_back(c.GetTypeByPath({"highway", "primary_link"}));
  }
};

class SecondaryChecker : public ftypes::BaseChecker
{
public:
  SecondaryChecker()
  {
    auto const & c = classif();

    m_types.push_back(c.GetTypeByPath({"highway", "secondary"}));
    m_types.push_back(c.GetTypeByPath({"highway", "secondary_link"}));
  }
};

class TertiaryChecker : public ftypes::BaseChecker
{
public:
  TertiaryChecker()
  {
    auto const & c = classif();
    m_types.push_back(c.GetTypeByPath({"highway", "tertiary"}));
    m_types.push_back(c.GetTypeByPath({"highway", "tertiary_link"}));
  }
};

class ResidentialChecker : public ftypes::BaseChecker
{
 public:
  ResidentialChecker()
  {
    auto const & c = classif();
    m_types.push_back(c.GetTypeByPath({"highway", "road"}));
    m_types.push_back(c.GetTypeByPath({"highway", "unclassified"}));
    m_types.push_back(c.GetTypeByPath({"highway", "residential"}));
  }
};

class LivingStreetChecker : public ftypes::BaseChecker
{
 public:
  LivingStreetChecker()
  {
    auto const & c = classif();
    m_types.push_back(c.GetTypeByPath({"highway", "living_street"}));
  }
};

class RoadInfoGetter
{
public:
  struct RoadInfo
  {
    openlr::FunctionalRoadClass m_frc;
    openlr::FormOfAWay m_fow;
  };

  RoadInfoGetter(Index const & index): m_index(index), m_c(classif()) {}

  /// Returns true if the |edge|'s road class is more important
  /// (stands higher in openlr::FunctionalRoadClass definition) than
  /// the restriction or equals to it.  Ex: FRC0 denotes a road with a
  /// higher importance than FRC1.
  bool PassesFRCLowestRestriction(Edge const & edge, openlr::FunctionalRoadClass const restriction)
  {
    if (edge.IsFake())
      return true;

    auto const frc = GetFeatureRoadInfo(edge.GetFeatureId()).m_frc;
    return frc <= restriction;
  }

private:
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

  openlr::FunctionalRoadClass GetFunctionalRoadClass(feature::TypesHolder const & types) const
  {
    if (m_trunkChecker(types))
      return openlr::FunctionalRoadClass::FRC0;

    if (m_primaryChecker(types))
      return openlr::FunctionalRoadClass::FRC1;

    if (m_secondaryChecker(types))
      return openlr::FunctionalRoadClass::FRC2;

    if (m_tertiaryChecker(types))
      return openlr::FunctionalRoadClass::FRC3;

    if (m_residentialChecker(types))
      return openlr::FunctionalRoadClass::FRC4;

    if (m_livingStreetChecker(types))
      return openlr::FunctionalRoadClass::FRC5;

    return openlr::FunctionalRoadClass::FRC7;
  }

  openlr::FormOfAWay GetFormOfAWay(feature::TypesHolder const & types) const
  {
    if (m_trunkChecker(types))
      return openlr::FormOfAWay::Motorway;

    if (m_primaryChecker(types))
      return openlr::FormOfAWay::MultipleCarriageway;

    return openlr::FormOfAWay::SingleCarriageway;
  }

  Index const & m_index;
  Classificator const & m_c;

  TrunkChecker const m_trunkChecker;
  PrimaryChecker const m_primaryChecker;
  SecondaryChecker const m_secondaryChecker;
  TertiaryChecker const m_tertiaryChecker;
  ResidentialChecker const m_residentialChecker;
  LivingStreetChecker const m_livingStreetChecker;

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

  bool Go(vector<InrixPoint> const & points, vector<routing::Edge> & path, double positiveOffsetM,
          double negativeOffsetM)
  {
    CHECK_GREATER_OR_EQUAL(points.size(), 2, ());

    m_graph.ResetFakes();

    m_pivots.clear();
    for (size_t i = 1; i + 1 < points.size(); ++i)
    {
      m_pivots.emplace_back();
      auto & ps = m_pivots.back();

      vector<pair<routing::Edge, Junction>> vicinity;
      m_graph.FindClosestEdges(points[i].m_point, kMaxRoadCandidates, vicinity);
      for (auto const & v : vicinity)
      {
        auto const & e = v.first;
        ps.push_back(e.GetStartJunction().GetPoint());
        ps.push_back(e.GetEndJunction().GetPoint());
      }

      if (ps.empty())
        return false;
    }
    m_pivots.push_back({points.back().m_point});
    CHECK_EQUAL(m_pivots.size() + 1, points.size(), ());

    Junction js(points.front().m_point, 0 /* altitude */);

    {
      vector<pair<routing::Edge, Junction>> sourceVicinity;
      m_graph.FindClosestEdges(points.front().m_point, kMaxRoadCandidates, sourceVicinity);
      m_graph.AddFakeEdges(js, sourceVicinity);
    }

    Junction jt(points.back().m_point, 0 /* altitude */);
    {
      vector<pair<routing::Edge, Junction>> targetVicinity;
      m_graph.FindClosestEdges(points.back().m_point, kMaxRoadCandidates, targetVicinity);
      m_graph.AddFakeEdges(jt, targetVicinity);
    }

    using State = pair<Score, Vertex>;
    priority_queue<State, vector<State>, greater<State>> queue;
    map<Vertex, Score> scores;
    Links links;

    auto pushVertex = [&queue, &scores, &links](Vertex const & u, Vertex const & v,
                                                Score const & sv, Edge const & e) {
      if ((scores.count(v) == 0 || scores[v].GetValue() > sv.GetValue() + kEps) && u != v)
      {
        scores[v] = sv;
        links[v] = make_pair(u, e);
        queue.emplace(sv, v);
      }
    };

    Vertex const s(js, js, 0 /* stageStartDistance */, 0 /* stage */, false /* bearingChecked */);
    scores[s] = Score();
    queue.emplace(scores[s], s);

    double const piS = GetPotential(s);

    while (!queue.empty())
    {
      auto const p = queue.top();
      queue.pop();

      Score const & su = p.first;
      Vertex const & u = p.second;

      if (su != scores[u])
        continue;

      if (u.m_stage == m_pivots.size())
      {
        vector<Edge> edges;
        auto cur = u;
        while (cur != s)
        {
          auto const & p = links[cur];

          edges.push_back(p.second);
          cur = p.first;
        }
        reverse(edges.begin(), edges.end());
        return ReconstructPath(points, edges, positiveOffsetM, negativeOffsetM, path);
      }

      size_t const stage = u.m_stage;
      double const distanceToNextPointM = points[stage].m_distanceToNextPointM;
      double const piU = GetPotential(u);
      double const ud = su.GetDistance() + piS - piU;  // real distance to u

      CHECK_LESS(stage, m_pivots.size(), ());

      // max(kDistanceAccuracyM, m_distanceToNextPointM) is added here
      // to throw out quite long paths.
      if (ud > u.m_stageStartDistance + distanceToNextPointM +
          max(kDistanceAccuracyM, distanceToNextPointM))
      {
        continue;
      }

      if (piU < kEps && !u.m_bearingChecked)
      {
        Vertex v = u;
        v.m_bearingChecked = true;

        Score sv = su;
        if (u.m_junction != u.m_stageStart)
        {
          int const expected = points[stage].m_bearing;
          int const actual = Bearing(u.m_stageStart.GetPoint(), u.m_junction.GetPoint());
          sv.AddBearingPenalty(expected, actual);
        }

        pushVertex(u, v, sv, Edge::MakeSpecial(u, v));
      }

      if (piU < kEps && u.m_bearingChecked)
      {
        Vertex v(u.m_junction, u.m_junction, ud /* stageStartDistance */, stage + 1,
                 false /* bearingChecked */);
        bool const isLastVertex = stage + 1 == m_pivots.size();

        double const piV = isLastVertex ? 0 : GetPotential(v);

        Score sv = su;
        sv.AddDistance(max(piV - piU, 0.0));
        sv.AddIntermediateErrorPenalty(
            MercatorBounds::DistanceOnEarth(u.m_junction.GetPoint(), points[stage + 1].m_point));

        if (isLastVertex)
        {
          int const expected = points.back().m_bearing;
          int const actual = GetReverseBearing(u, links);
          sv.AddBearingPenalty(expected, actual);
        }

        pushVertex(u, v, sv, Edge::MakeSpecial(u, v));

        if (isLastVertex)
          continue;
      }

      ForEachEdge(u, true /* outgoing */, points[stage].m_lfrcnp, [&](routing::Edge const & edge) {
        Vertex v(edge.GetEndJunction(), u.m_stageStart, u.m_stageStartDistance, stage,
                 u.m_bearingChecked);
        double const piV = GetPotential(v);

        Score sv = su;
        double const w = GetWeight(edge);
        sv.AddDistance(max(w + piV - piU, 0.0));

        double const vd = ud + w;  // real distance to v
        if (!v.m_bearingChecked && vd >= u.m_stageStartDistance + kBearingDist)
        {
          ASSERT_LESS(ud, u.m_stageStartDistance + kBearingDist, ());
          double const delta = vd - u.m_stageStartDistance - kBearingDist;
          auto const p = PointAtSegment(edge.GetStartJunction().GetPoint(),
                                        edge.GetEndJunction().GetPoint(), delta);
          if (u.m_stageStart.GetPoint() != p)
          {
            int const expected = points[stage].m_bearing;
            int const actual = Bearing(u.m_stageStart.GetPoint(), p);
            sv.AddBearingPenalty(expected, actual);
          }
          v.m_bearingChecked = true;
        }

        if (vd > v.m_stageStartDistance + distanceToNextPointM)
        {
          sv.AddDistanceErrorPenalty(
              std::min(vd - v.m_stageStartDistance - distanceToNextPointM, w));
        }

        if (edge.IsFake())
          sv.AddFakePenalty(w, edge.IsPartOfReal());

        pushVertex(u, v, sv, Edge::MakeNormal(u, v, edge));
      });
    }

    return false;
  }

private:
  struct Vertex
  {
    Vertex() = default;
    Vertex(Junction const & junction, Junction const & stageStart, double stageStartDistance,
           size_t stage, bool bearingChecked)
      : m_junction(junction)
      , m_stageStart(stageStart)
      , m_stageStartDistance(stageStartDistance)
      , m_stage(stage)
      , m_bearingChecked(bearingChecked)
    {
    }

    inline bool operator<(Vertex const & rhs) const
    {
      if (m_stage != rhs.m_stage)
        return m_stage < rhs.m_stage;
      if (m_junction != rhs.m_junction)
        return m_junction < rhs.m_junction;
      if (m_stageStart != rhs.m_stageStart)
        return m_stageStart < rhs.m_stageStart;
      return m_bearingChecked < rhs.m_bearingChecked;
    }

    inline bool operator==(Vertex const & rhs) const
    {
      return m_junction == rhs.m_junction && m_stageStart == rhs.m_stageStart &&
             m_stage == rhs.m_stage && m_bearingChecked == rhs.m_bearingChecked;
    }

    inline bool operator!=(Vertex const & rhs) const { return !(*this == rhs); }

    Junction m_junction;
    Junction m_stageStart;
    double m_stageStartDistance = 0.0;
    size_t m_stage = 0;
    bool m_bearingChecked = false;
  };

  struct Edge
  {
  public:
    Edge() = default;
    Edge(Vertex const & u, Vertex const & v, routing::Edge const & raw, bool isSpecial)
      : m_u(u), m_v(v), m_raw(raw), m_isSpecial(isSpecial)
    {
    }

    static Edge MakeNormal(Vertex const & u, Vertex const & v, routing::Edge const & raw)
    {
      return Edge(u, v, raw, false /* isSpecial */);
    }

    static Edge MakeSpecial(Vertex const & u, Vertex const & v)
    {
      return Edge(u, v, routing::Edge::MakeFake(u.m_junction, v.m_junction, false /* partOfReal */),
                  true /* isSpecial */);
    }

    inline bool IsFake() const { return m_raw.IsFake(); }

    inline bool IsSpecial() const { return m_isSpecial; }

    Vertex m_u;
    Vertex m_v;
    routing::Edge m_raw;
    bool m_isSpecial = false;
  };

  using Links = map<Vertex, pair<Vertex, Edge>>;

  class Score
  {
  public:
    // A weight for total length of true fake edges.
    static const int kTrueFakeCoeff = 3;

    // A weight for total length of fake edges that are parts of some
    // real edges.
    static constexpr double kFakeCoeff = 0.001;

    // A weight for passing too far from pivot points.
    static const int kIntermediateErrorCoeff = 3;

    // A weight for excess of distance limit.
    static const int kDistanceErrorCoeff = 3;

    // A weight for deviation from bearing.
    static const int kBearingErrorCoeff = 5;

    inline double GetDistance() const { return m_distance; }

    inline void AddDistance(double p)
    {
      m_distance += p;
      Update();
    }

    inline void AddFakePenalty(double p, bool partOfReal)
    {
      if (partOfReal)
        m_fakePenalty += p;
      else
        m_trueFakePenalty += p;
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
      Update();
    }

    inline double GetValue() const { return m_score; }

    bool operator<(Score const & rhs) const { return m_score < rhs.m_score; }
    bool operator>(Score const & rhs) const { return rhs < *this; }
    bool operator==(Score const & rhs) const { return m_score == rhs.m_score; }
    bool operator!=(Score const & rhs) const { return !(*this == rhs); }

  private:
    void Update()
    {
      m_score = m_distance + kTrueFakeCoeff * m_trueFakePenalty + kFakeCoeff * m_fakePenalty +
                kIntermediateErrorCoeff * m_intermediateErrorPenalty +
                kDistanceErrorCoeff * m_distanceErrorPenalty +
                kBearingErrorCoeff * m_bearingPenalty;
      CHECK_GREATER_OR_EQUAL(m_score, 0, ());
    }

    // Reduced length of path in meters.
    double m_distance = 0.0;

    // Penalty of passing by true fake edges.
    double m_trueFakePenalty = 0.0;

    // Penalty of passing by fake edges that are parts of some real edges.
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
    CHECK(!pivots.empty(), ("Empty list of pivots"));

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

  double GetWeight(routing::Edge const & e) const
  {
    return Distance(e.GetStartJunction(), e.GetEndJunction());
  }

  double GetWeight(Edge const & e) const { return GetWeight(e.m_raw); }

  bool PassesRestriction(routing::Edge const & edge,
                         openlr::FunctionalRoadClass const restriction) const
  {
    return m_roadInfoGetter.PassesFRCLowestRestriction(edge, restriction);
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
      auto const & edge = p.second.m_raw;

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
    return Bearing(a, b);
  }

  template <typename Fn>
  void ForEachEdge(Vertex const & u, bool outgoing, openlr::FunctionalRoadClass restriction,
                   Fn && fn)
  {
    IRoadGraph::TEdgeVector edges;
    if (outgoing)
      GetOutgoingEdges(u.m_junction, edges);
    else
      GetIngoingEdges(u.m_junction, edges);
    for (auto const & edge : edges)
    {
      if (!PassesRestriction(edge, restriction))
        continue;
      fn(edge);
    }
  }

  void GetOutgoingEdges(Junction const & u, IRoadGraph::TEdgeVector & edges)
  {
    GetEdges(u, &IRoadGraph::GetRegularOutgoingEdges, &IRoadGraph::GetFakeOutgoingEdges,
             m_outgoingCache, edges);
  }

  void GetIngoingEdges(Junction const & u, IRoadGraph::TEdgeVector & edges)
  {
    GetEdges(u, &IRoadGraph::GetRegularIngoingEdges, &IRoadGraph::GetFakeIngoingEdges,
             m_ingoingCache, edges);
  }

  void GetEdges(Junction const & u,
                void (IRoadGraph::*GetRegular)(Junction const & junction,
                                               IRoadGraph::TEdgeVector & edges) const,
                void (IRoadGraph::*GetFake)(Junction const & junction,
                                            IRoadGraph::TEdgeVector & edges) const,
                map<Junction, IRoadGraph::TEdgeVector> & cache, IRoadGraph::TEdgeVector & edges)
  {
    auto const it = cache.find(u);
    if (it == cache.end())
    {
      auto & es = cache[u];
      (m_graph.*GetRegular)(u, es);
      edges.insert(edges.end(), es.begin(), es.end());
    }
    else
    {
      auto const & es = it->second;
      edges.insert(edges.end(), es.begin(), es.end());
    }
    (m_graph.*GetFake)(u, edges);
  }

  template <typename Fn>
  void ForEachNonFakeEdge(Vertex const & u, bool outgoing, openlr::FunctionalRoadClass restriction,
                          Fn && fn)
  {
    ForEachEdge(u, outgoing, restriction, [&fn](routing::Edge const & edge) {
      if (!edge.IsFake())
        fn(edge);
    });
  }

  template <typename Fn>
  void ForEachNonFakeClosestEdge(Vertex const & u, openlr::FunctionalRoadClass const restriction,
                                 Fn && fn)
  {
    vector<pair<routing::Edge, Junction>> vicinity;
    m_graph.FindClosestEdges(u.m_junction.GetPoint(), kMaxRoadCandidates, vicinity);

    for (auto const & p : vicinity)
    {
      auto const & edge = p.first;
      if (edge.IsFake())
        continue;
      if (!PassesRestriction(edge, restriction))
        continue;
      fn(edge);
    }
  }

  template <typename It>
  size_t FindPrefixLengthToConsume(It b, It const e, double lengthM)
  {
    size_t n = 0;
    while (b != e && lengthM > 0.0)
    {
      auto const & u = b->first;
      auto const & v = b->second;
      double const len = MercatorBounds::DistanceOnEarth(u, v);
      if (2 * lengthM < len)
        break;

      lengthM -= len;
      ++n;
      ++b;
    }
    return n;
  }

  // Finds all edges that are on (u, v) and have the same direction as
  // (u, v).  Then, computes the fraction of the union of these edges
  // to the total length of (u, v).
  template <typename It>
  double GetCoverage(m2::PointD const & u, m2::PointD const & v, It b, It e)
  {
    double const kEps = 1e-5;

    m2::PointD const uv = v - u;
    double const sqlen = u.SquareLength(v);

    if (sqlen < kEps)
      return 0;

    vector<pair<double, double>> covs;
    for(; b != e; ++b)
    {
      auto const s = b->m_u.m_junction.GetPoint();
      auto const t = b->m_v.m_junction.GetPoint();
      if (!m2::IsPointOnSegmentEps(s, u, v, kEps) || !m2::IsPointOnSegmentEps(t, u, v, kEps))
        continue;

      if (DotProduct(uv, t - s) < -kEps)
        continue;

      double const sp = DotProduct(uv, s - u) / sqlen;
      double const tp = DotProduct(uv, t - u) / sqlen;

      double const start = my::clamp(min(sp, tp), 0, 1);
      double const finish = my::clamp(max(sp, tp), 0, 1);
      covs.emplace_back(start, finish);
    }

    sort(covs.begin(), covs.end());

    double coverage = 0;

    size_t i = 0;
    while (i != covs.size())
    {
      size_t j = i;

      double const first = covs[i].first;
      double last = covs[i].second;
      while (j != covs.size() && covs[j].first <= last)
      {
        last = max(last, covs[j].second);
        ++j;
      }

      coverage += last - first;
      i = j;
    }

    CHECK_LESS_OR_EQUAL(coverage, 1.0 + kEps, ());

    return coverage;
  }

  // Finds the longest prefix of [b, e) that covers edge (u, v).
  // Returns the fraction of the coverage to the length of the (u, v).
  template <typename It>
  double GetMatchingScore(m2::PointD const & u, m2::PointD const & v, It b, It e)
  {
    double const kEps = 1e-5;

    double const len = MercatorBounds::DistanceOnEarth(u, v);

    m2::PointD const uv = v - u;

    double cov = 0;
    for (; b != e; ++b)
    {
      auto const & s = b->first;
      auto const & t = b->second;
      if (!m2::IsPointOnSegmentEps(s, u, v, kEps) || !m2::IsPointOnSegmentEps(t, u, v, kEps))
        break;

      m2::PointD const st = t - s;
      if (DotProduct(uv, st) < -kEps)
        break;

      cov += MercatorBounds::DistanceOnEarth(s, t);
    }

    return len == 0 ? 0 : my::clamp(cov / len, 0, 1);
  }

  // Finds the longest prefix of fake edges of [b, e) that have the
  // same stage as |stage|. If the prefix exists, passes its bounding
  // iterator to |fn|.
  template <typename It, typename Fn>
  void ForStagePrefix(It b, It e, size_t stage, Fn && fn)
  {
    while (b != e && b->m_raw.IsFake() && b->m_u.m_stage == stage && b->m_v.m_stage == stage)
      ++b;
    if (b != e && !b->m_raw.IsFake())
      fn(b);
  }

  static pair<m2::PointD, m2::PointD> EdgeToPair(Edge const & edge)
  {
    auto const & e = edge.m_raw;
    return make_pair(e.GetStartJunction().GetPoint(), e.GetEndJunction().GetPoint());
  }

  static pair<m2::PointD, m2::PointD> EdgeToPairRev(Edge const & edge)
  {
    auto const & e = edge.m_raw;
    return make_pair(e.GetEndJunction().GetPoint(), e.GetStartJunction().GetPoint());
  }

  bool ReconstructPath(vector<InrixPoint> const & points, vector<Edge> & edges,
                       double positiveOffsetM, double negativeOffsetM, vector<routing::Edge> & path)
  {
    CHECK_GREATER_OR_EQUAL(points.size(), 2, ());

    using EdgeIt = vector<Edge>::iterator;
    using EdgeItRev = vector<Edge>::reverse_iterator;

    double const kFakeCoverageThreshold = 0.5;

    my::EraseIf(edges, mem_fn(&Edge::IsSpecial));

    {
      size_t const n = FindPrefixLengthToConsume(
          make_transform_iterator(edges.begin(), &EdgeToPair),
          make_transform_iterator(edges.end(), &EdgeToPair), positiveOffsetM);
      CHECK_LESS_OR_EQUAL(n, edges.size(), ());
      edges.erase(edges.begin(), edges.begin() + n);
    }

    {
      size_t const n = FindPrefixLengthToConsume(
          make_transform_iterator(edges.rbegin(), &EdgeToPairRev),
          make_transform_iterator(edges.rend(), &EdgeToPairRev), negativeOffsetM);
      CHECK_LESS_OR_EQUAL(n, edges.size(), ());
      edges.erase(edges.begin() + edges.size() - n, edges.end());
    }

    double frontEdgeScore = -1.0;
    routing::Edge frontEdge;
    ForStagePrefix(edges.begin(), edges.end(), 0, [&](EdgeIt e) {
      ForEachNonFakeEdge(e->m_u, false /* outgoing */, points[0].m_lfrcnp,
                         [&](routing::Edge const & edge) {
                           double const score = GetMatchingScore(
                               edge.GetEndJunction().GetPoint(), edge.GetStartJunction().GetPoint(),
                               make_transform_iterator(EdgeItRev(e), &EdgeToPairRev),
                               make_transform_iterator(edges.rend(), &EdgeToPairRev));
                           if (score > frontEdgeScore)
                           {
                             frontEdgeScore = score;
                             frontEdge = edge;
                           }
                         });
    });

    double backEdgeScore = -1.0;
    routing::Edge backEdge;
    ForStagePrefix(edges.rbegin(), edges.rend(), points.size() - 2 /* stage */, [&](EdgeItRev e) {
      ForEachNonFakeEdge(e->m_v, true /* outgoing */, points[points.size() - 2].m_lfrcnp,
                         [&](routing::Edge const & edge) {
                           double const score = GetMatchingScore(
                               edge.GetStartJunction().GetPoint(), edge.GetEndJunction().GetPoint(),
                               make_transform_iterator(e.base(), &EdgeToPair),
                               make_transform_iterator(edges.end(), &EdgeToPair));
                           if (score > backEdgeScore)
                           {
                             backEdgeScore = score;
                             backEdge = edge;
                           }
                         });
    });

    path.clear();
    for (auto const e : edges)
    {
      if (!e.IsFake())
        path.push_back(e.m_raw);
    }

    if (frontEdgeScore >= kFakeCoverageThreshold)
      path.insert(path.begin(), frontEdge);

    if (backEdgeScore >= kFakeCoverageThreshold)
      path.insert(path.end(), backEdge);

    if (path.empty())
    {
      // This is the case for routes from fake edges only.
      FindSingleEdgeApproximation(points, edges, path);
    }

    return !path.empty();
  }

  void FindSingleEdgeApproximation(vector<InrixPoint> const & points, vector<Edge> const & edges,
                                   vector<routing::Edge> & path)
  {
    double const kThreshold = 0.95;

    CHECK(all_of(edges.begin(), edges.end(), mem_fn(&Edge::IsFake)), ());

    double expectedLength = 0;
    for (auto const & edge : edges)
      expectedLength += GetWeight(edge);

    if (expectedLength < kEps)
      return;

    double bestCoverage = -1.0;
    routing::Edge bestEdge;

    auto checkEdge = [&](routing::Edge const & edge) {
      double const weight = GetWeight(edge);
      double const fraction =
          GetCoverage(edge.GetStartJunction().GetPoint(), edge.GetEndJunction().GetPoint(),
                      edges.begin(), edges.end());
      double const coverage = weight * fraction;
      if (fraction >= kThreshold && coverage >= bestCoverage)
      {
        bestCoverage = coverage;
        bestEdge = edge;
      }
    };

    for (auto const & edge : edges)
    {
      auto const & u = edge.m_u;
      auto const & v = edge.m_v;
      CHECK_EQUAL(u.m_stage, v.m_stage, ());
      auto const stage = u.m_stage;

      ForEachNonFakeClosestEdge(u, points[stage].m_lfrcnp, checkEdge);
      ForEachNonFakeClosestEdge(v, points[stage].m_lfrcnp, checkEdge);
    }

    if (bestCoverage >= expectedLength * kThreshold)
      path = {bestEdge};
  }

  FeaturesRoadGraph & m_graph;
  map<Junction, IRoadGraph::TEdgeVector> m_outgoingCache;
  map<Junction, IRoadGraph::TEdgeVector> m_ingoingCache;
  RoadInfoGetter & m_roadInfoGetter;
  vector<vector<m2::PointD>> m_pivots;
};

struct alignas(kCacheLineSize) Stats
{
  void Add(Stats const & rhs)
  {
    m_shortRoutes += rhs.m_shortRoutes;
    m_zeroCanditates += rhs.m_zeroCanditates;
    m_moreThanOneCandidates += rhs.m_moreThanOneCandidates;
    m_routeIsNotCalculated += rhs.m_routeIsNotCalculated;
    m_total += rhs.m_total;
  }

  uint32_t m_shortRoutes = 0;
  uint32_t m_zeroCanditates = 0;
  uint32_t m_moreThanOneCandidates = 0;
  uint32_t m_routeIsNotCalculated = 0;
  uint32_t m_total = 0;
};
}  // namespace

namespace openlr
{
// OpenLRSimpleDecoder::SegmentsFilter -------------------------------------------------------------
OpenLRSimpleDecoder::SegmentsFilter::SegmentsFilter(string const & idsPath,
                                                    bool const multipointsOnly)
  : m_idsSet(false), m_multipointsOnly(multipointsOnly)
{
  if (idsPath.empty())
    return;

  ifstream ifs(idsPath);
  CHECK(ifs, ("Can't find", idsPath));
  m_ids.insert(istream_iterator<uint32_t>(ifs), istream_iterator<uint32_t>());

  CHECK(!ifs, ("Garbage in", idsPath));
  m_idsSet = true;
}

bool OpenLRSimpleDecoder::SegmentsFilter::Matches(LinearSegment const & segment) const
{
  if (m_multipointsOnly && segment.m_locationReference.m_points.size() == 2)
    return false;
  if (m_idsSet && m_ids.count(segment.m_segmentId) == 0)
    return false;
  return true;
}

// OpenLRSimpleDecoder -----------------------------------------------------------------------------
int const OpenLRSimpleDecoder::kHandleAllSegments = -1;

OpenLRSimpleDecoder::OpenLRSimpleDecoder(string const & dataFilename, vector<Index> const & indexes)
  : m_indexes(indexes)
{
  auto const load_result = m_document.load_file(dataFilename.data());
  if (!load_result)
    MYTHROW(DecoderError, ("Can't load file", dataFilename, ":", load_result.description()));
}

void OpenLRSimpleDecoder::Decode(string const & outputFilename, int const segmentsToHandle,
                                 SegmentsFilter const & filter, int const numThreads)
{
  // TODO(mgsergio): Feed segments derectly to the decoder. Parsing sholud not
  // take place inside decoder process.
  vector<LinearSegment> segments;
  if (!ParseOpenlr(m_document, segments))
    MYTHROW(DecoderError, ("Can't parse data."));

  my::EraseIf(segments,
              [&filter](LinearSegment const & segment) { return !filter.Matches(segment); });

  if (segmentsToHandle != kHandleAllSegments && segmentsToHandle < segments.size())
    segments.resize(segmentsToHandle);

  sort(segments.begin(), segments.end(), my::LessBy(&LinearSegment::m_segmentId));

  vector<IRoadGraph::TEdgeVector> paths(segments.size());

  // This code computes the most optimal (in the sense of cache lines
  // occupancy) batch size.
  size_t constexpr a = LCM(sizeof(LinearSegment), kCacheLineSize) / sizeof(LinearSegment);
  size_t constexpr b =
      LCM(sizeof(IRoadGraph::TEdgeVector), kCacheLineSize) / sizeof(IRoadGraph::TEdgeVector);
  size_t constexpr kBatchSize = LCM(a, b);
  size_t constexpr kProgressFrequency = 100;

  auto worker = [&segments, &paths, kBatchSize, kProgressFrequency, numThreads, this](
      size_t threadNum, Index const & index, Stats & stats) {
    FeaturesRoadGraph roadGraph(index, IRoadGraph::Mode::ObeyOnewayTag,
                                make_unique<CarModelFactory>());
    RoadInfoGetter roadInfoGetter(index);
    AStarRouter astarRouter(roadGraph, roadInfoGetter);

    size_t const numSegments = segments.size();

    vector<InrixPoint> points;

    for (size_t i = threadNum * kBatchSize; i < numSegments; i += numThreads * kBatchSize)
    {
      for (size_t j = i; j < numSegments && j < i + kBatchSize; ++j)
      {
        auto const & segment = segments[j];
        auto const & ref = segment.m_locationReference;

        points.clear();
        for (auto const & point : ref.m_points)
          points.emplace_back(point);

        auto positiveOffsetM = ref.m_positiveOffsetMeters;
        if (points.size() == 2 && positiveOffsetM >= points[0].m_distanceToNextPointM)
        {
          LOG(LWARNING, ("Wrong positive offset for segment:", segment.m_segmentId));
          positiveOffsetM = 0;
        }

        auto negativeOffsetM = ref.m_negativeOffsetMeters;
        if (points.size() == 2 && negativeOffsetM >= points[0].m_distanceToNextPointM)
        {
          LOG(LWARNING, ("Wrong negative offset for segment:", segment.m_segmentId));
          negativeOffsetM = 0;
        }

        auto & path = paths[j];
        if (!astarRouter.Go(points, path, positiveOffsetM, negativeOffsetM))
          ++stats.m_routeIsNotCalculated;

        ++stats.m_total;

        if (stats.m_total % kProgressFrequency == 0)
          LOG(LINFO, ("Thread", threadNum, "processed:", stats.m_total));
      }
    }
  };

  vector<Stats> stats(numThreads);
  vector<thread> workers;
  for (size_t i = 1; i < numThreads; ++i)
    workers.emplace_back(worker, i, ref(m_indexes[i]), ref(stats[i]));
  worker(0 /* threadNum */, m_indexes[0], stats[0]);
  for (auto & worker : workers)
    worker.join();

  ofstream sample(outputFilename);
  for (size_t i = 0; i < segments.size(); ++i)
  {
    auto const & segment = segments[i];
    auto const & path = paths[i];

    if (path.empty())
      continue;

    sample << segment.m_segmentId << '\t';
    for (auto it = begin(path); it != end(path); ++it)
    {
      CHECK(!it->IsFake(), ("There should be no fake edges in the path."));
      auto const & fid = it->GetFeatureId();
      sample << fid.m_mwmId.GetInfo()->GetCountryName() << '-'
             << fid.m_index << '-' << it->GetSegId();
      sample << '-' << (it->IsForward() ? "fwd" : "bwd");
      sample << '-' << MercatorBounds::DistanceOnEarth(it->GetStartJunction().GetPoint(),
                                                       it->GetEndJunction().GetPoint());
      if (next(it) != end(path))
        sample << '=';
    }
    sample << endl;
  }

  Stats allStats;
  for (auto const & s : stats)
    allStats.Add(s);

  LOG(LINFO, ("Parsed segments:", allStats.m_total,
              "Routes failed:", allStats.m_routeIsNotCalculated,
              "Short routes:", allStats.m_shortRoutes,
              "Ambiguous routes:", allStats.m_moreThanOneCandidates,
              "Path is not reconstructed:", allStats.m_zeroCanditates));
}
}  // namespace openlr

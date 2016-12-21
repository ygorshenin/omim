#pragma once

#include "openlr/way_point.hpp"

#include "routing/road_graph.hpp"

#include "geometry/point2d.hpp"

#include "std/map.hpp"
#include "std/utility.hpp"
#include "std/vector.hpp"

namespace routing
{
class FeaturesRoadGraph;
}

namespace openlr
{
class RoadInfoGetter;

class Router final
{
public:
  Router(routing::FeaturesRoadGraph & graph, RoadInfoGetter & roadInfoGetter);

  bool Go(vector<WayPoint> const & points, vector<routing::Edge> & path, double positiveOffsetM,
          double negativeOffsetM);

private:
  struct Vertex final
  {
    Vertex() = default;
    Vertex(routing::Junction const & junction, routing::Junction const & stageStart,
           double stageStartDistance, size_t stage, bool bearingChecked);

    bool operator<(Vertex const & rhs) const;
    bool operator==(Vertex const & rhs) const;
    inline bool operator!=(Vertex const & rhs) const { return !(*this == rhs); }

    routing::Junction m_junction;
    routing::Junction m_stageStart;
    double m_stageStartDistance = 0.0;
    size_t m_stage = 0;
    bool m_bearingChecked = false;
  };

  struct Edge final
  {
    Edge() = default;
    Edge(Vertex const & u, Vertex const & v, routing::Edge const & raw, bool isSpecial);

    static Edge MakeNormal(Vertex const & u, Vertex const & v, routing::Edge const & raw);
    static Edge MakeSpecial(Vertex const & u, Vertex const & v);

    inline bool IsFake() const { return m_raw.IsFake(); }
    inline bool IsSpecial() const { return m_isSpecial; }

    pair<m2::PointD, m2::PointD> ToPair() const;
    pair<m2::PointD, m2::PointD> ToPairRev() const;

    Vertex m_u;
    Vertex m_v;
    routing::Edge m_raw;
    bool m_isSpecial = false;
  };

  using Links = map<Vertex, pair<Vertex, Edge>>;

  using RoadGraphEdgesGetter = void (routing::IRoadGraph::*)(
      routing::Junction const & junction, routing::IRoadGraph::TEdgeVector & edges) const;

  double GetPotential(Vertex const & u) const;

  inline double GetWeight(routing::Edge const & e) const
  {
    return MercatorBounds::DistanceOnEarth(e.GetStartJunction().GetPoint(),
                                           e.GetEndJunction().GetPoint());
  }

  inline double GetWeight(Edge const & e) const { return GetWeight(e.m_raw); }

  bool PassesRestriction(routing::Edge const & edge, FunctionalRoadClass const restriction) const;

  uint32_t GetReverseBearing(Vertex const & u, Links const & links) const;

  template <typename Fn>
  void ForEachEdge(Vertex const & u, bool outgoing, FunctionalRoadClass restriction, Fn && fn);

  void GetOutgoingEdges(routing::Junction const & u, routing::IRoadGraph::TEdgeVector & edges);
  void GetIngoingEdges(routing::Junction const & u, routing::IRoadGraph::TEdgeVector & edges);
  void GetEdges(routing::Junction const & u, RoadGraphEdgesGetter getRegular,
                RoadGraphEdgesGetter getFake,
                map<routing::Junction, routing::IRoadGraph::TEdgeVector> & cache,
                routing::IRoadGraph::TEdgeVector & edges);

  template <typename Fn>
  void ForEachNonFakeEdge(Vertex const & u, bool outgoing, FunctionalRoadClass restriction,
                          Fn && fn);

  template <typename Fn>
  void ForEachNonFakeClosestEdge(Vertex const & u, FunctionalRoadClass const restriction, Fn && fn);

  template <typename It>
  size_t FindPrefixLengthToConsume(It b, It const e, double lengthM);

  // Finds all edges that are on (u, v) and have the same direction as
  // (u, v).  Then, computes the fraction of the union of these edges
  // to the total length of (u, v).
  template <typename It>
  double GetCoverage(m2::PointD const & u, m2::PointD const & v, It b, It e);

  // Finds the longest prefix of [b, e) that covers edge (u, v).
  // Returns the fraction of the coverage to the length of the (u, v).
  template <typename It>
  double GetMatchingScore(m2::PointD const & u, m2::PointD const & v, It b, It e);

  // Finds the longest prefix of fake edges of [b, e) that have the
  // same stage as |stage|. If the prefix exists, passes its bounding
  // iterator to |fn|.
  template <typename It, typename Fn>
  void ForStagePrefix(It b, It e, size_t stage, Fn && fn);

  bool ReconstructPath(vector<WayPoint> const & points, vector<Edge> & edges,
                       double positiveOffsetM, double negativeOffsetM,
                       vector<routing::Edge> & path);

  void FindSingleEdgeApproximation(vector<WayPoint> const & points, vector<Edge> const & edges,
                                   vector<routing::Edge> & path);

  routing::FeaturesRoadGraph & m_graph;
  map<routing::Junction, routing::IRoadGraph::TEdgeVector> m_outgoingCache;
  map<routing::Junction, routing::IRoadGraph::TEdgeVector> m_ingoingCache;
  RoadInfoGetter & m_roadInfoGetter;
  vector<vector<m2::PointD>> m_pivots;
};
}  // namespace openlr

#include "../../testing/testing.hpp"

#include "road_graph_builder.hpp"
#include "features_road_graph_test.hpp"

#include "../dijkstra_router.hpp"
#include "../features_road_graph.hpp"
#include "../route.hpp"

#include "../../indexer/classificator_loader.hpp"

#include "../../base/logging.hpp"
#include "../../base/macros.hpp"


using namespace routing;
using namespace routing_test;


// Use mock graph source.
template <size_t finalPosSize, size_t startPosSize, size_t expectedSize>
void TestDijkstraRouterMock(RoadPos (&finalPos)[finalPosSize],
                            RoadPos (&startPos)[startPosSize],
                            RoadPos (&expected)[expectedSize])
{
  RoadGraphMockSource * graph = new RoadGraphMockSource();
  InitRoadGraphMockSourceWithTest2(*graph);

  DijkstraRouter router;

  router.SetRoadGraph(graph);

  vector<RoadPos> result;
  router.CalculateM2MRoute(vector<RoadPos>(&startPos[0], &startPos[0] + ARRAY_SIZE(startPos)),
                           vector<RoadPos>(&finalPos[0], &finalPos[0] + ARRAY_SIZE(finalPos)),
                           result);
  TEST_EQUAL(vector<RoadPos>(&expected[0], &expected[0] + ARRAY_SIZE(expected)), result, ());
}

// Use mwm features graph source.
template <size_t finalPosSize, size_t startPosSize, size_t expectedSize>
void TestDijkstraRouterMWM(RoadPos (&finalPos)[finalPosSize],
                           RoadPos (&startPos)[startPosSize],
                           RoadPos (&expected)[expectedSize],
                           size_t pointsCount)
{
  FeatureRoadGraphTester tester("route_test2.mwm");

  DijkstraRouter router;
  router.SetRoadGraph(tester.GetGraph());

  vector<RoadPos> finalV(&finalPos[0], &finalPos[0] + ARRAY_SIZE(finalPos));
  tester.Name2FeatureID(finalV);

  vector<RoadPos> startV(&startPos[0], &startPos[0] + ARRAY_SIZE(startPos));
  tester.Name2FeatureID(startV);

  vector<RoadPos> result;
  router.CalculateM2MRoute(startV, finalV, result);
  LOG(LDEBUG, (result));

  Route route(router.GetName());
  tester.GetGraph()->ReconstructPath(result, route);
  LOG(LDEBUG, (route));
  TEST_EQUAL(route.GetPoly().GetSize(), pointsCount, ());

  tester.FeatureID2Name(result);
  TEST_EQUAL(vector<RoadPos>(&expected[0], &expected[0] + ARRAY_SIZE(expected)), result, ());
}


UNIT_TEST(Dijkstra_Router_City_Simple)
{
  classificator::Load();

  // Uncomment to see debug log.
  //my::g_LogLevel = LDEBUG;

  RoadPos finalPos[] = { RoadPos(7, true, 0) };
  RoadPos startPos[] = { RoadPos(1, true, 0) };

  RoadPos expected1[] = { RoadPos(1, true, 0),
                          RoadPos(1, true, 1),
                          RoadPos(8, true, 0),
                          RoadPos(8, true, 1),
                          RoadPos(7, true, 0) };
  TestDijkstraRouterMock(finalPos, startPos, expected1);

  RoadPos expected2[] = { RoadPos(1, true, 0),
                          RoadPos(1, true, 1),
                          RoadPos(8, true, 1),
                          RoadPos(7, true, 0) };
  TestDijkstraRouterMWM(finalPos, startPos, expected2, 4);
}


UNIT_TEST(Dijkstra_Router_City_ReallyFunnyLoop)
{
  classificator::Load();

  // Uncomment to see debug log.
  //my::g_LogLevel = LDEBUG;

  RoadPos finalPos[] = { RoadPos(1, true, 0) };
  RoadPos startPos[] = { RoadPos(1, true, 1) };
  RoadPos expected1[] = { RoadPos(1, true, 1),
                          RoadPos(8, true, 0),
                          RoadPos(8, true, 1),
                          RoadPos(8, true, 3), // algorithm skips 8,2 segment
                          RoadPos(4, false, 0),
                          RoadPos(0, false, 1),
                          RoadPos(0, false, 0),
                          RoadPos(1, true, 0) };
  TestDijkstraRouterMock(finalPos, startPos, expected1);

  RoadPos expected2[] = { RoadPos(1, true, 1),
                          RoadPos(8, true, 4),
                          RoadPos(2, true, 1),
                          RoadPos(0, false, 0),
                          RoadPos(1, true, 0) };
  TestDijkstraRouterMWM(finalPos, startPos, expected2, 9);

}

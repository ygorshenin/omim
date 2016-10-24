#include "openlr/openlr_simple_decoder.hpp"

#include "routing/car_model.hpp"
#include "routing/car_router.hpp"
#include "routing/features_road_graph.hpp"
#include "routing/router_delegate.hpp"

#include "indexer/index.hpp"
#include "indexer/scales.hpp"

#include "geometry/polyline2d.hpp"

#include "base/logging.hpp"
#include "base/math.hpp"

#include "geometry/latlon.hpp"

// double constexpr kMwmRoadCrossingRadiusMeters = 2.0;
// m2::RectD const rect = MercatorBounds::RectByCenterXYAndSizeInMeters(cross, kMwmRoadCrossingRadiusMeters);
//   m_index.ForEachInRect(featuresLoader, rect, GetStreetReadScale());
// }

namespace  // Primitive utilities to handle simple INRI OpenLR XML data.
{
bool GetLatLon(pugi::xml_node node, int32_t & lat, int32_t & lon)
{
  node = node.child("olr:coordinate");
  auto const latNode = node.child("olr:latitude");
  if (!latNode)
    return false;

  auto const lonNode = node.child("olr:longitude");
  if (!lonNode)
    return false;

  lat = latNode.text().as_int();
  lon = lonNode.text().as_int();

  return true;
}

pugi::xml_node GetLinearLocationReference(pugi::xml_node const & node)
{
  // TODO(mgsergio): Check format (how many optional linear location reference childred
  // are in loation reference tag).
  auto const locations =
      node.select_nodes(".//olr:locationReference/olr:optionLinearLocationReference");
  // TODO(mgsergio): remove me.
  ASSERT_EQUAL(locations.size(), 1, ());

  return node.select_node(".//olr:locationReference/olr:optionLinearLocationReference").node();
}

bool GetSegmentId(pugi::xml_node const & node, uint64_t & id)
{
  auto const idNode = node.child("ReportSegmentID");
  if (!idNode)
    return false;
  id = idNode.text().as_uint();
  return true;
}
}  // namespace

namespace  // OpenLR tools and abstractions
{
bool GetFirstCoordinate(pugi::xml_node const & node, ms::LatLon & latLon)
{
  int32_t lat, lon;
  if (!GetLatLon(node, lat, lon))
    return false;

  latLon.lat = ((lat - (my::Sign(lat) * 0.5)) * 360) / (1 << 24);
  latLon.lon = ((lon - (my::Sign(lon) * 0.5)) * 360) / (1 << 24);

  return true;
}

bool GetCoordinate(pugi::xml_node const & node, ms::LatLon const & firstCoord, ms::LatLon & latLon)
{
  int32_t lat, lon;
  if (!GetLatLon(node, lat, lon))
    return false;

  latLon.lat = firstCoord.lat + static_cast<double>(lat) / 100000;
  latLon.lon = firstCoord.lon + static_cast<double>(lon) / 100000;

  return true;
}
}  // namespace

namespace  // A staff to get road data.
{
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
                                           routing::FeaturesRoadGraph & roadGraph,
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

routing::IRoadGraph::TEdgeVector GetOutgoingEdges(routing::FeaturesRoadGraph const & roadGraph,
                                                  ms::LatLon const & latLon)
{
  routing::IRoadGraph::TEdgeVector edges;

  routing::Junction junction(MercatorBounds::FromLatLon(latLon), feature::kDefaultAltitudeMeters);
  roadGraph.GetOutgoingEdges(junction, edges);

  return edges;
}

routing::IRoadGraph::TEdgeVector GetIngoingEdges(routing::FeaturesRoadGraph const & roadGraph,
                                                 ms::LatLon const & latLon)
{
  routing::IRoadGraph::TEdgeVector edges;

  routing::Junction junction(MercatorBounds::FromLatLon(latLon), feature::kDefaultAltitudeMeters);
  roadGraph.GetIngoingEdges(junction, edges);

  return edges;
}
}  // namespace

namespace openlr
{
OpenLRSimpleDecoder::OpenLRSimpleDecoder(string const & dataFilename, Index const & index,
                                         routing::IRouter & router)
  : m_index(index)
  , m_router(router)
{
  auto const load_result = m_document.load_file(dataFilename.data());
  if (!load_result)
    MYTHROW(DecoderError, ("Can't load file", dataFilename, ":", load_result.description()));
}

void OpenLRSimpleDecoder::Decode()
{
  routing::FeaturesRoadGraph roadGraph(m_index, routing::IRoadGraph::Mode::ObeyOnewayTag,
                                       make_unique<routing::CarModelFactory>());

  uint32_t count = 0;
  for (auto const xpath_node : m_document.select_nodes("//reportSegments"))
  {
    auto const node = xpath_node.node();
    uint64_t segmentID;
    if (!GetSegmentId(node, segmentID))
    {
      LOG(LERROR, ("Cant't parse segment id"));
      continue;
    }

    auto const locRefNode = GetLinearLocationReference(node);
    if (!locRefNode)
    {
      LOG(LERROR, ("Can't get loaction reference"));
      continue;
    }

    if (locRefNode.child("olr:intermediates"))
    {
      LOG(LDEBUG, ("Intermediate points encounted. Skipping the whole segment."));
      continue;
    }

    ms::LatLon first, last;
    if (!GetFirstCoordinate(locRefNode.child("olr:first"), first))
    {
      LOG(LERROR, ("Can't get firts coordinate"));
      continue;
    }

    if (!GetCoordinate(locRefNode.child("olr:last"), first, last))
    {
      LOG(LERROR, ("Can't get last coordinate"));
      continue;
    }

    if (first.EqualDxDy(last, 1e-4))
    {
      LOG(LDEBUG, ("Frist and last points are to close: ", first, last));
      continue;
    }

    ++count;

    // auto const outgoingEdges = GetOutgoingEdges(roadGraph, first);
    // auto const ingoingEdges = GetIngoingEdges(roadGraph, last);

    // LOG(LINFO, ("Number of outgoing enges:", outgoingEdges.size(), "for point: ", first));
    // LOG(LINFO, ("Number of ingoing enges:", ingoingEdges.size(), "for point: ", last));

    // auto const featuresForFirst = GetRoadFeaturesAtPoint(m_index, roadGraph, first);
    // auto const featuresForLast = GetRoadFeaturesAtPoint(m_index, roadGraph, last);

    /// Calculate route
    first = ms::LatLon(55.712814774096798942, 37.381468803977099924);
    last = ms::LatLon(55.713313757543446059, 37.384826761461006583);
    routing::RouterDelegate delegate;
    auto route = make_shared<routing::Route>("openlr");
    auto const result =
        m_router.CalculateRoute(MercatorBounds::FromLatLon(first), {0.0, 0.0},
                                MercatorBounds::FromLatLon(last), delegate, *route.get());
    if (result != routing::IRouter::ResultCode::NoError)
    {
      LOG(LDEBUG, ("Can't calculate route for points", first, last, ", code:", result));
      continue;
    }

    return;
    /// ---

    // cout << "Features: " << featuresForFirst.size() << "\tfor point" << DebugPrint(first) << endl;
    // cout << "Features: " << featuresForLast.size() << "\tfor point" << DebugPrint(last) << endl;

    // LOG(LINFO, ("Number of features", featuresForFirst.size(), "for first point: ", first));
    // LOG(LINFO, ("Number of features", featuresForLast.size(), "for last point: ", last));
  }

  LOG(LINFO, (count, "Segments found"));
}
}  // namespace openlr

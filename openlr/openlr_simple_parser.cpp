#include "openlr/openlr_simple_parser.hpp"

#include "base/logging.hpp"

#include "3party/pugixml/src/pugixml.hpp"

namespace  // Primitive utilities to handle simple INRIX OpenLR XML data.
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
  auto locations =
      node.select_nodes(".//olr:locationReference");
  // TODO(mgsergio): remove me.
  ASSERT_EQUAL(locations.size(), 1, ());

  locations =
      node.select_nodes(".//olr:locationReference/olr:optionLinearLocationReference");
  // TODO(mgsergio): remove me.
  ASSERT_EQUAL(locations.size(), 1, ());

  return node.select_node(".//olr:locationReference/olr:optionLinearLocationReference").node();
}

bool GetSegmentId(pugi::xml_node const & node, uint32_t & id)
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

  latLon.lat = ((lat - my::Sign(lat) * 0.5) * 360) / (1 << 24);
  latLon.lon = ((lon - my::Sign(lon) * 0.5) * 360) / (1 << 24);

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

bool ParseLocationReferencePoint(pugi::xml_node const & locPointNode,
                                 openlr::LocationReferencePoint & locPoint)
{
  if (!GetFirstCoordinate(locPointNode, locPoint.m_latLon))
  {
    LOG(LERROR, ("Can't get first coordinate"));
    return false;
  }

  return true;
}

bool ParseLocationReferencePoint(pugi::xml_node const & locPointNode, ms::LatLon const & firstPoint,
                                 openlr::LocationReferencePoint & locPoint)
{
  if (!GetCoordinate(locPointNode, firstPoint, locPoint.m_latLon))
  {
    LOG(LERROR, ("Can't get last coordinate"));
    return false;
  }

  return true;
}

bool ParseLinearLocationReference(pugi::xml_node const & locRefNode,
                                  openlr::LinearLocationReference & locRef)
{
  if (!locRefNode)
  {
    LOG(LERROR, ("Can't get loaction reference"));
    return false;
  }

  // TODO(mgsergio): Handle intermediate points.
  if (locRefNode.child("olr:intermediates"))
  {
    LOG(LDEBUG, ("Intermediate points encounted. Skipping the whole segment."));
    return false;
  }

  {
    openlr::LocationReferencePoint point;
    if (!ParseLocationReferencePoint(locRefNode.child("olr:first"), point))
      return false;
    locRef.m_points.push_back(point);
  }

  // We relay on we get olr::last really last in this loop. Since it shoud be last in
  // olr::optionalLinearLocationReference and select_nodes should not change the order of
  // node in the document.
  // TODO(mgsergio): Test intermediate case.
  for (auto const pointNode : locRefNode.select_nodes("olr:intermediates|olr:last"))
  {
    openlr::LocationReferencePoint point;
    if (!ParseLocationReferencePoint(pointNode.node(), locRef.m_points.front().m_latLon, point))
      return false;
    locRef.m_points.push_back(point);
  }

  // TODO(mgsergio): Either remove or move to loop above.
  // if (first.EqualDxDy(last, 1e-4))
  // {
  //   LOG(LDEBUG, ("Frist and last points are to close: ", first, last));
  //   return false;
  // }
  return true;
}

bool ParseSegment(pugi::xml_node const & segmentNode, openlr::Segment & segment)
{
  auto const node = segmentNode;
  if (!GetSegmentId(node, segment.m_segmentId))
  {
    LOG(LERROR, ("Can't parse segment id"));
    return false;
  }

  // TODO(mgsergio): Parse segmentLengthMetes;

  auto const locRefNode = GetLinearLocationReference(node);
  return ParseLinearLocationReference(locRefNode, segment.m_locationReference);
}
}  // namespace

namespace openlr
{
bool ParseOpenlr(pugi::xml_document const & document, vector<Segment> & segments)
{
  for (auto const segmentXpathNode : document.select_nodes("//reportSegments"))
  {
    Segment segment;
    if (!ParseSegment(segmentXpathNode.node(), segment))
      return false;
    segments.push_back(segment);
  }
  return true;
}
}  // namespace openlr

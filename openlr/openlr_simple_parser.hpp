#pragma once

#include "geometry/latlon.hpp"

namespace pugi
{
class xml_document;
};

namespace openlr
{
struct LocationReferencePoint
{
  ms::LatLon m_latLon;
  // FunctinalRoadClass
  // FormOfAWay
  // Bearing
};

struct LinearLocationReference
{
  vector<LocationReferencePoint> m_points;
  uint32_t m_positiveOffsetMeters;
  uint32_t m_negativeOffsetMeters;
};

struct Segment
{
  uint32_t m_segmentId;
  // TODO(mgsergio): Make sure that one segment cannot contain
  // more than one location reference.
  LinearLocationReference m_locationReference;
  uint32_t m_segmentLengthMeters;
  // uint32_t m_segmentRefSpeed;  Always null in Inrix data. (No docs found).
};

bool ParseOpenlr(pugi::xml_document const & document, vector<Segment> & segments);
}  // namespace openlr

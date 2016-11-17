#pragma once

#include "geometry/latlon.hpp"
#include "geometry/point2d.hpp"

namespace pugi
{
class xml_document;
}  // namespace pugi

// TODO(mgsergio): Move all these structures to a saparated file.
namespace openlr
{
enum class FormOfAWay
{
  // The physical road type is unknown.
  UNDEFINED,
  // A road permitted for motorized vehicles only in combination with a prescribed minimum speed.
  // It has two or more physically separated carriageways and no single level-crossings.
  MOTORWAY,
  // A road with physically separated carriageways regardless of the number of lanes.
  // If a road is also a motorway, it should be coded as such and not as a multiple carriageway.
  MULTIPLE_CARRIAGEWAY,
  // All roads without separate carriageways are considered as roads with a single carriageway.
  SINGLE_CARRIAGEWAY,
  // A road which forms a ring on which traffic traveling in only one direction is allowed.
  ROUNDABOUT,
  // An open area (partly) enclosed by roads which is used for non-traffic purposes
  // and which is not a Roundabout.
  TRAFFICSQUARE,
  // A road especially designed to enter or leave a line.
  SLIPROAD,
  // The physical road type is known, but does not fit into one of the other categories.
  OTHER,
  // A path only allowed for bikes.
  BIKE_PATH,
  // A path only allowed for pedestrians.
  FOOTPATH
};

enum class FunctionalRoadClass
{

  // Main road, highest importance.
  FRC0,
  // First class road.
  FRC1,

  // Other classes.

  FRC2,
  FRC3,
  FRC4,
  FRC5,
  FRC6,
  FRC7
};

// TODO(mgsergio): Add comments.
struct LocationReferencePoint
{
  ms::LatLon m_latLon;
  uint8_t m_bearing;
  FunctionalRoadClass m_functionalRoadClass;
  FormOfAWay m_formOfAWay;
};

struct LinearLocationReference
{
  vector<LocationReferencePoint> m_points;
  uint32_t m_positiveOffsetMeters;
  uint32_t m_negativeOffsetMeters;
};

struct LinearSegment
{
  vector<m2::PointD> GetMercatorPoints() const;

  // TODO(mgsergio): Think of using openlr::PartnerSegmentId
  uint32_t m_segmentId;
  // TODO(mgsergio): Make sure that one segment cannot contain
  // more than one location reference.
  LinearLocationReference m_locationReference;
  uint32_t m_segmentLengthMeters;
  // uint32_t m_segmentRefSpeed;  Always null in Inrix data. (No docs found).
};

bool ParseOpenlr(pugi::xml_document const & document, vector<LinearSegment> & segments);
}  // namespace openlr

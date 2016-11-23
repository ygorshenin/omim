#pragma once

#include "geometry/latlon.hpp"
#include "geometry/point2d.hpp"

#include "std/limits.hpp"

namespace openlr
{
enum class FormOfAWay
{
  // The physical road type is unknown.
  Undefined,
  // A road permitted for motorized vehicles only in combination with a prescribed minimum speed.
  // It has two or more physically separated carriageways and no single level-crossings.
  Motorway,
  // A road with physically separated carriageways regardless of the number of lanes.
  // If a road is also a motorway, it should be coded as such and not as a multiple carriageway.
  MultipleCarriageway,
  // All roads without separate carriageways are considered as roads with a single carriageway.
  SingleCarriageway,
  // A road which forms a ring on which traffic traveling in only one direction is allowed.
  Roundabout,
  // An open area (partly) enclosed by roads which is used for non-traffic purposes
  // and which is not a Roundabout.
  Trafficsquare,
  // A road especially designed to enter or leave a line.
  Sliproad,
  // The physical road type is known, but does not fit into one of the other categories.
  Other,
  // A path only allowed for bikes.
  BikePath,
  // A path only allowed for pedestrians.
  Footpath,
  NotAValue
};

enum class FunctionalRoadClass
{
  // Main road, highest importance.
  FRC0,
  // First class road.
  FRC1,

  // Other road classes.

  FRC2,
  FRC3,
  FRC4,
  FRC5,
  FRC6,
  FRC7,
  NotAValue
};

// TODO(mgsergio): Add comments.
struct LocationReferencePoint
{
  ms::LatLon m_latLon;
  uint8_t m_bearing;
  FunctionalRoadClass m_functionalRoadClass = FunctionalRoadClass::NotAValue;
  FormOfAWay m_formOfAWay = FormOfAWay::NotAValue;

  // Should not be used in the last point of a segment.
  uint32_t m_distanceToNextPoint = 0;
  // Should not be used in the last point of a segment.
  FunctionalRoadClass m_lfrcnp = FunctionalRoadClass::NotAValue;
  bool m_againstDrivingDirection = false;
};

struct LinearLocationReference
{
  vector<LocationReferencePoint> m_points;
  uint32_t m_positiveOffsetMeters = 0;
  uint32_t m_negativeOffsetMeters = 0;
};

struct LinearSegment
{
  static auto constexpr kInvalidSegmentId = numeric_limits<uint32_t>::max();

  vector<m2::PointD> GetMercatorPoints() const;

  // TODO(mgsergio): Think of using openlr::PartnerSegmentId
  uint32_t m_segmentId = kInvalidSegmentId;
  // TODO(mgsergio): Make sure that one segment cannot contain
  // more than one location reference.
  LinearLocationReference m_locationReference;
  uint32_t m_segmentLengthMeters = 0;
  // uint32_t m_segmentRefSpeed;  Always null in Inrix data. (No docs found).
};
}  // namespace openlr

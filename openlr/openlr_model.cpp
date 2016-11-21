#include "geometry/mercator.hpp"

#include "openlr/openlr_model.hpp"

namespace openlr
{
// LinearSegment -----------------------------------------------------------------------------------
vector<m2::PointD> LinearSegment::GetMercatorPoints() const
{
  vector<m2::PointD> points;
  auto const & referencePoints = m_locationReference.m_points;
  for (auto const & point : referencePoints)
    points.push_back(MercatorBounds::FromLatLon(point.m_latLon));
  return points;
}
}  // namespace openlr

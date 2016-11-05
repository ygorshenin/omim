#include "openlr/traffic_mode.hpp"

namespace openlr
{
TrafficMode::TrafficMode(string const & dataFileName, string const & sampleFileName,
                         Index const & index)
  : m_index(index)
{
}
}  // namespace openlr

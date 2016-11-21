#pragma once

#include "std/vector.hpp"

namespace pugi
{
class xml_document;
}  // namespace pugi

// TODO(mgsergio): Move all these structures to a saparated file.
namespace openlr
{
struct LinearSegment;
bool ParseOpenlr(pugi::xml_document const & document, vector<LinearSegment> & segments);
}  // namespace openlr

#pragma once

#include "base/exception.hpp"

#include "std/string.hpp"

#include "3party/pugixml/src/pugixml.hpp"

class Index;

namespace openlr
{
DECLARE_EXCEPTION(DecoderError, RootException);

class OpenLRSimpleDecoder
{
public:
  static int const kHandleAllSegmets;

  OpenLRSimpleDecoder(string const & dataFilename, Index const & index);

  void Decode(int segmentsTohandle, bool multipointsOnly);

private:
  Index const & m_index;
  pugi::xml_document m_document;
};
}  // namespace openlr

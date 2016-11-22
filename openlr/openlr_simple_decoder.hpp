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
  static int const kHandleAllSegments;

  OpenLRSimpleDecoder(string const & dataFilename, Index const & index);

  void Decode(string const & outputFilename, int segmentsToHandle, bool multipointsOnly,
              int numThreads);

private:
  Index const & m_index;
  pugi::xml_document m_document;
};
}  // namespace openlr

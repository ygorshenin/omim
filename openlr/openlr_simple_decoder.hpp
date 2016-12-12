#pragma once

#include "base/exception.hpp"

#include "std/string.hpp"
#include "std/vector.hpp"

#include "3party/pugixml/src/pugixml.hpp"

class Index;

namespace openlr
{
struct LinearSegment;

DECLARE_EXCEPTION(DecoderError, RootException);

class OpenLRSimpleDecoder
{
public:
  class SegmentsFilter
  {
  public:
    SegmentsFilter(string const & idsPath, bool const multipointsOnly);

    bool Matches(LinearSegment const & segment) const;

  private:
    unordered_set<uint32_t> m_ids;
    bool m_idsSet;
    bool const m_multipointsOnly;
  };

  static int const kHandleAllSegments;

  OpenLRSimpleDecoder(string const & dataFilename, vector<Index> const & indexes);

  void Decode(string const & outputFilename, int segmentsToHandle, SegmentsFilter const & filter,
              int numThreads);

private:
  vector<Index> const & m_indexes;
  pugi::xml_document m_document;
};
}  // namespace openlr

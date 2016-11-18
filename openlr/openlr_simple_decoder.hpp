#pragma once

#include "base/exception.hpp"

#include "std/string.hpp"

#include "3party/pugixml/src/pugixml.hpp"

class Index;

namespace routing
{
class IRouter;
}  // namespace routing

namespace openlr
{
DECLARE_EXCEPTION(DecoderError, RootException);


class OpenLRSimpleDecoder
{
public:
  static int const kHandleAllSegmets;

  OpenLRSimpleDecoder(string const & dataFilename, Index const & index,
                      routing::IRouter & router);

  void Decode(int segmentsTohandle = kHandleAllSegmets);

private:
  Index const & m_index;
  routing::IRouter & m_router;
  pugi::xml_document m_document;

};
}  // namespace openlr

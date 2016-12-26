#pragma once

#include "indexer/feature_decl.hpp"

#include "base/exception.hpp"
#include "base/newtype.hpp"

#include "std/vector.hpp"
#include "std/string.hpp"

class Index;

namespace openlr
{
NEWTYPE(uint32_t, PartnerSegmentId);

enum class ItemEvaluation
{
  Unevaluated,
  Positive,
  Negative,
  RelPositive,
  RelNegative,
  Ignore
};

struct SampleItem
{
  struct MWMSegment
  {
    MWMSegment(FeatureID const & fid, uint32_t const segId, bool const isForward)
      : m_fid(fid)
      , m_segId(segId)
      , m_isForward(isForward)
    {
    }

    // TODO(mgsergio): switch to osm id.
    FeatureID const m_fid;
    uint32_t const m_segId;
    bool const m_isForward;
  };

  PartnerSegmentId m_partnerSegmentId;
  vector<MWMSegment> m_segments;
  // May become a number later.
  ItemEvaluation m_evaluation;
};

DECLARE_EXCEPTION(SamplePoolLoadError, RootException);
DECLARE_EXCEPTION(SamplePoolSaveError, RootException);

using SamplePool = vector<SampleItem>;

SamplePool LoadSamplePool(string const & fileName, Index const & index);
void SaveSamplePool(string const & fileName, SamplePool const & sample);
}  // namespace openlr

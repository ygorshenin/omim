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
  Negative
};

struct SampleItem
{
  struct MWMSegemnt
  {
    // TODO(mgsergio): switch to osm id.
    FeatureID m_fid;
    uint32_t m_segId;
  };

  PartnerSegmentId m_partnerSegmentId;
  vector<MWMSegemnt> m_segments;
  // May become a number later.
  ItemEvaluation m_evaluation;
};

DECLARE_EXCEPTION(SamplePoolLoadError, RootException);
DECLARE_EXCEPTION(SamplePoolSaveError, RootException);

using SamplePool = vector<SampleItem>;

SamplePool LoadSamplePool(string const & fileName, Index const & index);
void SaveSamplePool(string const & fileName, SamplePool const & sample);
}  // namespace openlr

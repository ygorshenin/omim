#pragma once

#include "indexer/feature.hpp"

#include "openlr/openlr_sample.hpp"

#include "std/string.hpp"

class Index;

namespace openlr
{
struct Segment
{
  FeatureID m_fid;
  uint32_t m_segId;
};

struct DecodedSampleItem
{
  PartnerSegmentId m_parterSegmentId;
  vector<Segment> segments;
};

class DecodedSample
{
public:
  DecodedSample(Index const & index, SamplePool const & sample);

private:
  map<FeatureID, FeatureType> m_features;
  vector<DecodedSampleItem> m_decodedItems;
};

/// This class is used to map sample ids to real data
/// and change sample evaluations.
class TrafficMode
{
public:
  TrafficMode(string const & dataFileName, string const & sampleFileName, Index const & index);

private:
  Index const & m_index;
  // DecodedSample m_decodedSample;
};
}  // namespace openlr
// m_trafficSample = openlr::LoadSamplePool(fileName, m_pDrawWidget->GetFramework().GetIndex());

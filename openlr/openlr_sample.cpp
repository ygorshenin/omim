#include "openlr/openlr_sample.hpp"

#include "indexer/index.hpp"

#include "base/string_utils.hpp"

#include "std/cerrno.hpp"
#include "std/cstring.hpp"
#include "std/fstream.hpp"
#include "std/string.hpp"

namespace
{
void ParseMWMSegments(string const & line, uint32_t const lineNumber,
                      vector<openlr::SampleItem::MWMSegemnt> & segments, Index const & index)
{
  vector<string> parts;
  strings::ParseCSVRow(line, '=', parts);

  for (auto const seg : parts)
  {
    vector<string> segParts;
    strings::ParseCSVRow(seg, '-', segParts);
    ASSERT_EQUAL(segParts.size(), 3, ());

    auto const mwmId = index.GetMwmIdByCountryFile(platform::CountryFile(segParts[0]));

    uint32_t featureIndex;
    if (!strings::to_uint(segParts[1], featureIndex))
      MYTHROW(openlr::SamplePoolLoadError, ("Can't parse MWMSegment", seg, "line:", lineNumber));

    uint32_t segId;
    if (!strings::to_uint(segParts[2], segId))
      MYTHROW(openlr::SamplePoolLoadError, ("Can't parse MWMSegment", seg, "line:", lineNumber));

    segments.push_back({FeatureID(mwmId, featureIndex), segId});
  }
}

void ParseSampleItem(string const & line, uint32_t const lineNumber, openlr::SampleItem & item,
                     Index const & index)
{
  vector<string> parts;
  strings::ParseCSVRow(line, '\t', parts);
  ASSERT_GREATER_OR_EQUAL(parts.size(), 2, ());
  ASSERT_LESS_OR_EQUAL(parts.size(), 3, ());

  auto nextFieldIndex = 0;
  if (parts.size() == 3)
  {
    if (parts[nextFieldIndex] == "P")
      item.m_evaluation = openlr::ItemEvaluation::Positive;
    else if (parts[nextFieldIndex] == "N")
      item.m_evaluation = openlr::ItemEvaluation::Negative;
    else if (parts[nextFieldIndex] == "I")
      item.m_evaluation = openlr::ItemEvaluation::Ignore;
    // TODO(mgsergio): Handle wrong format separately.
    else
      item.m_evaluation = openlr::ItemEvaluation::Unevaluated;
    ++nextFieldIndex;
  }
  else
  {
    item.m_evaluation = openlr::ItemEvaluation::Unevaluated;
  }

  if (!strings::to_uint(parts[nextFieldIndex], item.m_partnerSegmentId.Get()))
  {
    MYTHROW(openlr::SamplePoolLoadError, ("Error: can't parse field", nextFieldIndex,
                                          "(number expected) in line:", lineNumber));
  }
  ++nextFieldIndex;

  ParseMWMSegments(parts[nextFieldIndex], lineNumber, item.m_segments, index);
}
}  // namepsace

namespace openlr
{
SamplePool LoadSamplePool(string const & fileName, Index const & index)
{
  ifstream sample(fileName);
  if (!sample.is_open())
    MYTHROW(SamplePoolLoadError, ("Can't read form file", fileName, strerror(errno)));

  SamplePool pool;
  for (struct {uint32_t lineNumber = 0; string line; } st; getline(sample, st.line); ++st.lineNumber)
  {
    SampleItem item;
    ParseSampleItem(st.line, st.lineNumber, item, index);
    pool.push_back(item);
  }

  return pool;
}

void SaveSamplePool(string const & fileName, SamplePool const & sample)
{
  LOG(LDEBUG, ("Saving sample to a file:", fileName));
  ofstream out(fileName);
  if (!out.is_open())
    MYTHROW(SamplePoolSaveError, ("Can't write to file", fileName, strerror(errno)));
  for (auto const & item : sample)
  {
    switch (item.m_evaluation)
    {
    case ItemEvaluation::Unevaluated: out << 'U'; break;
    case ItemEvaluation::Positive: out << 'P'; break;
    case ItemEvaluation::Negative: out << 'N'; break;
    case ItemEvaluation::Ignore: out << 'I'; break;
    }
    out << '\t';

    out << item.m_partnerSegmentId.Get() << '\t';

    // TODO(mgsergio): Couple with same code from openlr_simple_decoder.
    for (auto it = begin(item.m_segments); it != end(item.m_segments); ++it)
    {
      auto const & fid = it->m_fid;
      out << fid.m_mwmId.GetInfo()->GetCountryName() << '-'
          << fid.m_index << '-' << it->m_segId;
      if (next(it) != end(item.m_segments))
        out << '=';
    }
    out << endl;
  }
  if (out.fail())
    MYTHROW(SamplePoolSaveError, ("An error occur while writing file", fileName, strerror(errno)));
}
}  // namespace openlr

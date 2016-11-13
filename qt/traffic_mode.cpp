#include "qt/traffic_mode.hpp"

#include "openlr/openlr_simple_parser.hpp"

#include "indexer/index.hpp"

#include "3party/pugixml/src/pugixml.hpp"

#include <QItemSelection>

// DecodedSample -----------------------------------------------------------------------------------
DecodedSample::DecodedSample(Index const & index, openlr::SamplePool const & sample)
{
  for (auto const & item : sample)
  {
    m_decodedItems.emplace_back();
    auto & decodedItem = m_decodedItems.back();
    decodedItem.m_partnerSegmentId = item.m_partnerSegmentId;
    decodedItem.m_evaluation = item.m_evaluation;
    for (auto const & mwmSegment : item.m_segments)
    {
      auto const & fid = mwmSegment.m_fid;
      Index::FeaturesLoaderGuard g(index, fid.m_mwmId);
      CHECK(fid.m_mwmId.IsAlive(), ("Mwm id is not alive."));
      if (m_features.find(fid) == end(m_features))
      {
        auto & ft = m_features[fid];
        CHECK(g.GetFeatureByIndex(fid.m_index, ft), ("Can't read feature", fid));
        ft.ParseEverything();
      }
      decodedItem.m_segments.push_back({mwmSegment.m_fid, mwmSegment.m_segId});
    }
  }
}

// TrfficMode --------------------------------------------------------------------------------------
TrafficMode::TrafficMode(string const & dataFileName, string const & sampleFileName,
                         Index const & index, unique_ptr<ITrafficDrawerDelegate> drawerDelagate,
                         QObject * parent)
  : QAbstractTableModel(parent)
  , m_drawerDelegate(move(drawerDelagate))
{
  try
  {
    auto const & sample = openlr::LoadSamplePool(sampleFileName, index);
    m_decodedSample = make_unique<DecodedSample>(index, sample);
  }
  catch (openlr::SamplePoolLoadError const & e)
  {
    LOG(LERROR, (e.Msg()));
    return;
  }

  pugi::xml_document doc;
  if (!doc.load_file(dataFileName.data()))
  {
    LOG(LERROR, ("Can't load file:", dataFileName));
    return;
  }

  vector<openlr::LinearSegment> segments;
  if (!ParseOpenlr(doc, segments))
  {
    LOG(LERROR, ("Can't parse data:", dataFileName));
    return;
  }
  for (auto const & segment : segments)
  {
    m_partnerSegments[segment.m_segmentId] = segment;
  }

  m_valid = true;
}

int TrafficMode::rowCount(const QModelIndex & parent) const
{
  if (!m_decodedSample)
    return 0;
  return m_decodedSample->m_decodedItems.size();
}

int TrafficMode::columnCount(const QModelIndex & parent) const
{
  return 2;
}

QVariant TrafficMode::data(const QModelIndex & index, int role) const
{
  if (!index.isValid())
    return QVariant();

  if (index.row() >= rowCount())
    return QVariant();

  if (role != Qt::DisplayRole)
    return QVariant();

  if (index.column() == 0)
    return "Unevaluated";//m_decodedSample->m_decodedItems[index.row()].m_evaluation;

  if (index.column() == 1)
    return m_decodedSample->m_decodedItems[index.row()].m_partnerSegmentId.Get();

  return QVariant();
}

void TrafficMode::OnItemSelected(QItemSelection const & selected, QItemSelection const &)
{
  ASSERT(!selected.empty(), ("The selection should not be empty. RTFM for qt5."));
  auto const row = selected.front().top();
  // TODO(mgsergio): Use algo for center calculation.
  // Now viewport is set to the first point of the first segment.
  auto const partnerSegmentId = m_decodedSample->m_decodedItems[row].m_partnerSegmentId;
  auto const & firstSegment = m_decodedSample->m_decodedItems[row].m_segments[0];
  auto const & firstSegmentFeatureId = firstSegment.m_fid;
  auto const & firstSegmentFeature = m_decodedSample->m_features.at(firstSegmentFeatureId);

  LOG(LDEBUG, ("PartnerSegmentId:", partnerSegmentId.Get(),
               "Segment points:", m_partnerSegments[partnerSegmentId.Get()].GetMercatorPoints(),
               "Featrue segment id", firstSegment.m_segId,
               "Feature segment points", firstSegmentFeature.GetPoint(firstSegment.m_segId),
                                         firstSegmentFeature.GetPoint(firstSegment.m_segId + 1)));

  m_drawerDelegate->Clear();
  m_drawerDelegate->SetViewportCenter(firstSegmentFeature.GetPoint(firstSegment.m_segId));
  m_drawerDelegate->DrawEncodedSegment(m_partnerSegments[partnerSegmentId.Get()]);
  m_drawerDelegate->DrawDecodedSegments(*m_decodedSample, row);
}

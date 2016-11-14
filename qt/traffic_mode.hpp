#pragma once

#include "indexer/feature.hpp"

#include "openlr/openlr_sample.hpp"
#include "openlr/openlr_simple_parser.hpp"

#include "std/string.hpp"
#include "std/unique_ptr.hpp"
#include "std/unordered_map.hpp"

#include <QAbstractTableModel>

class QItemSelection;
class Index;

// TODO(mgsergio): Use openlr::SampleItem::MWMSegment instead.
struct Segment
{
  FeatureID m_fid;
  uint32_t m_segId;
};

// TODO(mgsergio): Use openlr::SampleItem.
struct DecodedSampleItem
{
  openlr::PartnerSegmentId m_partnerSegmentId;
  vector<Segment> m_segments;
  openlr::ItemEvaluation m_evaluation;
};

struct DecodedSample
{
  DecodedSample(Index const & index, openlr::SamplePool const & sample);

  openlr::SamplePool ToOpenlrSamplePool() const;

  map<FeatureID, FeatureType> m_features;
  vector<DecodedSampleItem> m_decodedItems;
};

/// This class is used to delegate segments drawing to the DrapeEngine.
class ITrafficDrawerDelegate
{
public:
  virtual ~ITrafficDrawerDelegate() = default;

  virtual void SetViewportCenter(m2::PointD const & center) = 0;

  virtual void DrawDecodedSegments(DecodedSample const & sample, int sampleIndex) = 0;
  virtual void DrawEncodedSegment(openlr::LinearSegment const & segment) = 0;
  virtual void Clear() = 0;
};

/// This class is used to map sample ids to real data
/// and change sample evaluations.
class TrafficMode : public QAbstractTableModel
{
  Q_OBJECT

public:
  TrafficMode(string const & dataFileName, string const & sampleFileName, Index const & index,
              unique_ptr<ITrafficDrawerDelegate> drawerDelagate, QObject * parent = Q_NULLPTR);

  bool SaveSampleAs(string const & fileName) const;
  bool IsValid() const { return m_valid; }

  int rowCount(const QModelIndex & parent = QModelIndex()) const Q_DECL_OVERRIDE;
  int columnCount(const QModelIndex & parent = QModelIndex()) const Q_DECL_OVERRIDE;

  QVariant data(const QModelIndex & index, int role) const Q_DECL_OVERRIDE;
  // QVariant headerData(int section, Qt::Orientation orientation,
  //                     int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;

  Qt::ItemFlags flags(QModelIndex const & index) const Q_DECL_OVERRIDE;
  bool setData(QModelIndex const & index, QVariant const & value, int role = Qt::EditRole) Q_DECL_OVERRIDE;

public slots:
  void OnItemSelected(QItemSelection const & selected, QItemSelection const &);

private:
  unique_ptr<DecodedSample> m_decodedSample;
  // TODO(mgsergio): use map instead of vector.
  unordered_map<decltype(openlr::LinearSegment::m_segmentId), openlr::LinearSegment> m_partnerSegments;

  unique_ptr<ITrafficDrawerDelegate> m_drawerDelegate;

  bool m_valid = false;
};

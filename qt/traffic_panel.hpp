#pragma once

#include <QWidget>
#include <QTableWidget>

namespace openlr
{
class TrafficMode;
}  // namespace openlr

class TrafficPanel : public QWidget
{
  QTableWidget * m_table;
  openlr::TrafficMode & m_trafficMode;

  Q_OBJECT

public:
  explicit TrafficPanel(openlr::TrafficMode & trafficMode, QWidget * parent);

private:
  void CreateTable();
  void FillTable();

signals:

public slots:
  void OnRowClicked(QItemSelection const & selected, QItemSelection const &);
  void OnCheckBoxClicked(int row, int state);
};

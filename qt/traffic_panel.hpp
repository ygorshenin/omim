#pragma once

#include <QWidget>

class QAbstractItemModel;
class QTableView;

class TrafficPanel : public QWidget
{
  Q_OBJECT

public:
  explicit TrafficPanel(QAbstractItemModel * trafficModel, QWidget * parent);

private:
  void CreateTable(QAbstractItemModel * trafficModel);
  void FillTable();

signals:

public slots:
  // void OnCheckBoxClicked(int row, int state);

private:
  QTableView * m_table;
};

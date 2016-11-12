#include "qt/traffic_panel.hpp"
#include "qt/traffic_mode.hpp"

#include "std/string.hpp"

#include <QAbstractTableModel>
#include <QBoxLayout>
#include <QCheckBox>
#include <QHeaderView>
#include <QLabel>
#include <QTableView>

TrafficPanel::TrafficPanel(QAbstractItemModel * trafficModel, QWidget * parent)
  : QWidget(parent)
{
  CreateTable(trafficModel);
  FillTable();

  auto layout = new QVBoxLayout();
  layout->addWidget(m_table);
  setLayout(layout);
}

void TrafficPanel::CreateTable(QAbstractItemModel * trafficModel)
{
  m_table = new QTableView();
  m_table->setFocusPolicy(Qt::NoFocus);
  m_table->setAlternatingRowColors(true);
  m_table->setShowGrid(false);
  m_table->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  m_table->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);
  m_table->verticalHeader()->setVisible(false);
  m_table->horizontalHeader()->setVisible(false);
  m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  m_table->setModel(trafficModel);

  connect(m_table->selectionModel(),
          SIGNAL(selectionChanged(QItemSelection const &, QItemSelection const &)),
          trafficModel, SLOT(OnItemSelected(QItemSelection const &, QItemSelection const &)));
}

void TrafficPanel::FillTable()
{
  // for (auto const & item : m_samplePool)
  // {
  //   // TODO(mgsergio): Collaps segments so thay can be shown in table.
  //   auto const row = m_table->rowCount();
  //   m_table->insertRow(row);

  //   auto checkBox = new QCheckBox(tr("Valid"));
  //   checkBox->setCheckState(item.m_evaluation ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
  //   connect(checkBox, &QCheckBox::stateChanged, [this, row](int const state)
  //   {
  //     emit OnCheckBoxClicked(row, state);
  //   });
  //   m_table->setCellWidget(row, 0, checkBox);

  //   auto const partnerId = QString::number(item.m_parterSegmentId.Get());
  //   m_table->setCellWidget(row, 1, new QLabel(partnerId));
  // }
}

// void TrafficPanel::OnCheckBoxClicked(int row, int state)
// {
//   // if (state == Qt::CheckState::Unchecked)
//   // {
//   //   m_samplePool[row].m_evaluation = false;
//   //   return;
//   // }
//   // m_samplePool[row].m_evaluation = true;
// }

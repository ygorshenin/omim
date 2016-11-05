#include "qt/traffic_panel.hpp"

#include "std/string.hpp"

#include <QBoxLayout>
#include <QCheckBox>
#include <QHeaderView>
#include <QLabel>

TrafficPanel::TrafficPanel(openlr::TrafficMode & trafficMode, QWidget * parent)
  : QWidget(parent)
  , m_trafficMode(trafficMode)
{
  CreateTable();
  FillTable();

  auto layout = new QVBoxLayout();
  layout->addWidget(m_table);
  setLayout(layout);
}

void TrafficPanel::CreateTable()
{
  m_table = new QTableWidget(0, 2, this);
  m_table->setFocusPolicy(Qt::NoFocus);
  m_table->setAlternatingRowColors(true);
  m_table->setShowGrid(false);
  m_table->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  m_table->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);
  m_table->verticalHeader()->setVisible(false);
  m_table->horizontalHeader()->setVisible(false);
  m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  connect(m_table->selectionModel(),
          SIGNAL(selectionChanged(QItemSelection const &, QItemSelection const &)),
          this, SLOT(OnRowClicked(QItemSelection const &, QItemSelection const &)));
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

void TrafficPanel::OnCheckBoxClicked(int row, int state)
{
  // if (state == Qt::CheckState::Unchecked)
  // {
  //   m_samplePool[row].m_evaluation = false;
  //   return;
  // }
  // m_samplePool[row].m_evaluation = true;
}

// TODO(mgsergio): Remove me.
#include "base/logging.hpp"
void TrafficPanel::OnRowClicked(QItemSelection const & selected, QItemSelection const &)
{
  for (auto const ind : selected.indexes())
    LOG(LINFO, ("clicked:", ind.row(), ind.column()));
}

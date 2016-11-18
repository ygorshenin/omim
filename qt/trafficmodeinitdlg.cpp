#include "trafficmodeinitdlg.h"
#include "ui_trafficmodeinitdlg.h"

#include "platform/settings.hpp"

#include <QFileDialog>

namespace
{
string const kDataFilePath = "LastTrafficDataFilePath";
string const kSampleFilePath = "LastTrafficSampleFilePath";
}  // namespace

TrafficModeInitDlg::TrafficModeInitDlg(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::TrafficModeInitDlg)
{
  ui->setupUi(this);

  string lastDataFilePath;
  string lastSampleFilePath;
  if (settings::Get(kDataFilePath, lastDataFilePath) &&
      settings::Get(kSampleFilePath, lastSampleFilePath))
  {
    ui->dataFileName->setText(QString::fromStdString(lastDataFilePath));
    ui->sampleFileName->setText(QString::fromStdString(lastSampleFilePath));
  }

  connect(ui->chooseDataFileButton, &QPushButton::clicked, [this](bool)
  {
    SetFilePathViaDialog(*ui->dataFileName, tr("Choose traffic data file"), "*.xml");
  });
  connect(ui->chooseSampleFileButton, &QPushButton::clicked, [this](bool)
  {
    SetFilePathViaDialog(*ui->sampleFileName, tr("Choose traffic sample file"));
  });
}

TrafficModeInitDlg::~TrafficModeInitDlg()
{
  delete ui;
}

void TrafficModeInitDlg::accept()
{
  m_dataFileName = ui->dataFileName->text().trimmed().toStdString();
  m_sampleFileName = ui->sampleFileName->text().trimmed().toStdString();

  settings::Set(kDataFilePath, m_dataFileName);
  settings::Set(kSampleFilePath, m_sampleFileName);

  QDialog::accept();
}

void TrafficModeInitDlg::SetFilePathViaDialog(QLineEdit & dest, QString const & title,
                                              QString const & filter)
{
  QFileDialog openFileDlg(nullptr, title, {} /* directory */, filter);
  openFileDlg.exec();
  if (openFileDlg.result() != QDialog::DialogCode::Accepted)
    return;

  dest.setText(openFileDlg.selectedFiles().first());
}

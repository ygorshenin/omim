#include "trafficmodeinitdlg.h"
#include "ui_trafficmodeinitdlg.h"

#include <QFileDialog>

TrafficModeInitDlg::TrafficModeInitDlg(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::TrafficModeInitDlg)
{
  ui->setupUi(this);

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

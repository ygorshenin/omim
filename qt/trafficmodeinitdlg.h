#pragma once

#include "std/string.hpp"

#include <QDialog>

class QLineEdit;

namespace Ui {
class TrafficModeInitDlg;
}

class TrafficModeInitDlg : public QDialog
{
  Q_OBJECT

public:
  explicit TrafficModeInitDlg(QWidget *parent = 0);
  ~TrafficModeInitDlg();

  string GetDataFilePath() const { return m_dataFileName; }
  string GetSampleFilePath() const { return m_sampleFileName; }

private:
  void SetFilePathViaDialog(QLineEdit & dest, QString const & title,
                            QString const & filter = {});
public slots:
  void accept() override;

private:
  Ui::TrafficModeInitDlg *ui;

  string m_dataFileName;
  string m_sampleFileName;
};

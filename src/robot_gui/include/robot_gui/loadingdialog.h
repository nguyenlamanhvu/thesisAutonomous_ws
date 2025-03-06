#ifndef LOADINGDIALOG_H
#define LOADINGDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QMovie>
#include <QVBoxLayout>

namespace Ui {
class LoadingDialog;
}

class LoadingDialog : public QDialog
{
  Q_OBJECT

public:
  explicit LoadingDialog(QWidget *parent = nullptr);
  ~LoadingDialog();

private:
  Ui::LoadingDialog *ui;
  QLabel *loadingLabel;
  QMovie *loadingMovie;
};

#endif // LOADINGDIALOG_H

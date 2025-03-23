#ifndef LOADINGDIALOG_H
#define LOADINGDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QMovie>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QMessageBox>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace Ui {
class LoadingDialog;
}

class LoadingDialog : public QDialog
{
  Q_OBJECT

public:
  explicit LoadingDialog(QWidget *parent = nullptr);
  ~LoadingDialog();
  bool isClosed() const { return closed; }
  void userNotForceClose(){forceClose = false;}

protected:
  void closeEvent(QCloseEvent *event) override;

private:
  Ui::LoadingDialog *ui;
  QLabel *loadingLabel;
  QMovie *loadingMovie;
  bool closed = false;
  bool forceClose = true;
  ros::NodeHandle nh;
  ros::Publisher stop_GA_pub;
  std_msgs::Bool stopFlag;
};

#endif // LOADINGDIALOG_H

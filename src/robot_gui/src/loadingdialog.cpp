#include "loadingdialog.h"
#include "../../../build/robot_gui/ui_loadingdialog.h"

LoadingDialog::LoadingDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::LoadingDialog)
{
//  ui->setupUi(this);
  setWindowTitle("Loading...");
  setModal(true);

  loadingLabel = new QLabel(this);
  loadingLabel->setAlignment(Qt::AlignCenter);

  loadingMovie = new QMovie(":/Icons/Loading.gif");  // Use an animated GIF
  loadingLabel->setMovie(loadingMovie);
  loadingMovie->start();

  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->addWidget(loadingLabel);
  setLayout(layout);

  stop_GA_pub = nh.advertise<std_msgs::Bool>("/GA_stop_flag", 10);
  stopFlag.data = false;
  stop_GA_pub.publish(stopFlag);
}

LoadingDialog::~LoadingDialog()
{
//  delete ui;
  loadingMovie->stop();
}

void LoadingDialog::closeEvent(QCloseEvent *event) {
    if(!closed && !forceClose)
    {
       event->accept();  // Allow closing
       closed = true;
       forceClose = true;
       stopFlag.data = false;
       stop_GA_pub.publish(stopFlag);
       return;
    }
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Exit", "Are you sure you want to stop finding optimize path?",
                                  QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        closed = true;
        stopFlag.data = true;
        stop_GA_pub.publish(stopFlag);
        event->accept();  // Allow closing
    } else {
        forceClose = false;
        stopFlag.data = false;
        stop_GA_pub.publish(stopFlag);
        event->ignore();  // Prevent closing
    }
}

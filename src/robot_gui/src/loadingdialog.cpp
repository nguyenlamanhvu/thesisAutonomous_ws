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
}

LoadingDialog::~LoadingDialog()
{
//  delete ui;
  loadingMovie->stop();
}

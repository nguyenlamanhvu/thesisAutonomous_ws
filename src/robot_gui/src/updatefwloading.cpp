#include "updatefwloading.h"
#include "../../../build/robot_gui/ui_updatefwloading.h"

updateFWLoading::updateFWLoading(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::updateFWLoading)
{
  //  ui->setupUi(this);
    setWindowTitle("Loading...");
    setModal(true);

    loadingLabel = new QLabel(this);
    loadingLabel->setAlignment(Qt::AlignCenter);

    loadingMovie = new QMovie(":/Icons/installing_updates.gif");
    loadingLabel->setMovie(loadingMovie);
    loadingMovie->start();

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(loadingLabel);
    setLayout(layout);
}

updateFWLoading::~updateFWLoading()
{
//  delete ui;
    loadingMovie->stop();
}

#include "robot_gui.h"
#include "ui_robot_gui.h"

robot_gui::robot_gui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::robot_gui)
{
  ui->setupUi(this);
}

robot_gui::~robot_gui()
{
  delete ui;
}

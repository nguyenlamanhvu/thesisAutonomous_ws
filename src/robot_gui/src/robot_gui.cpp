#include "robot_gui.h"
#include "ui_robot_gui.h"
#include <QIcon>

robot_gui::robot_gui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::robot_gui)
{
  QIcon icon(":/Icons/Search_Icon.png");
//  ui->btSearch->setIcon(icon);
  ui->setupUi(this);
}

robot_gui::~robot_gui()
{
  delete ui;
}

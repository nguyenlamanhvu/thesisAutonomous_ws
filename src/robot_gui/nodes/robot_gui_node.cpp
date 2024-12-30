#include <QApplication>
#include "robot_gui.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robot_gui_node", ros::init_options::AnonymousName);
  QApplication a(argc, argv);

  robot_gui w;
  w.setWindowTitle(QString::fromStdString(ros::this_node::getName()));
  w.show();
  return a.exec();
}

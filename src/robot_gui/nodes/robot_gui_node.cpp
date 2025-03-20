#include <QApplication>
#include "robot_gui.h"
#include <QIcon>
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robot_gui_node", ros::init_options::AnonymousName);
  QApplication a(argc, argv);

  robot_gui robotGui;
  robotGui.setWindowTitle(QString::fromStdString(ros::this_node::getName()));

#if (USE_JETSON_NANO)
  robotGui.setWindowFlags(Qt::FramelessWindowHint);
  robotGui.showFullScreen();
#endif
  QIcon icon(":/Icons/my_gui_icon.png");
  robotGui.setWindowIcon(icon);
  robotGui.show();
  return a.exec();
}

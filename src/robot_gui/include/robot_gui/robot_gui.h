#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include <QWidget>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace Ui {
class robot_gui;
}

class robot_gui : public QWidget
{
  Q_OBJECT

public:
  explicit robot_gui(QWidget *parent = nullptr);
  ~robot_gui();

private:
  Ui::robot_gui *ui;
};

#endif // ROBOT_GUI_H

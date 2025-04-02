#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include "compiler_switch.h"
#include "../../../robot_navigation/include/robot_navigation/json.hpp"
#include <QWidget>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <robot_navigation/GARequest.h>
#include <robot_update/updateFWSignal.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <actionlib_msgs/GoalID.h>
#include <QDateTime>
#include <QDebug>
#include <QTimer>
#include <QVector>
#include <QVBoxLayout>
#include <QTableWidgetItem>
#include <QLineEdit>
#include <QListWidget>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>
#include <QPointF>
#include <QDir>
#include <QString>
#include <QRegularExpression>
#include <QProcess>

#if USE_MAP_RVIZ
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/display.h>
#include <rviz/properties/property.h>
#include <rviz/properties/float_property.h>
#elif !USE_MAP_RVIZ
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <QPoint>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QResizeEvent>
#include <QSize>
#include <QPinchGesture>
#include <QGesture>
#endif

namespace Ui {
class robot_gui;
}

class robot_gui : public QWidget
{
  Q_OBJECT

public:
  explicit robot_gui(QWidget *parent = nullptr);
  ~robot_gui();
  enum TableRowProduct
  {
    index =           0,
    listOfProducts =  1,
    choosenProducts = 2,
  };

  struct fileNameData {
      QString fileName;
      std::string filePath;
      QVector<double> position;
      QVector<double> orientation;
  };

  void loadProductName();
  void finish_flag_callback(const std_msgs::Bool::ConstPtr &msg);
  void robot_velocity_callback(const geometry_msgs::Twist::ConstPtr &msg);
  void footprint_callback(const geometry_msgs::PolygonStamped::ConstPtr &msg);
  void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void killRosLaunch(const QString &launchName);
  void startRosLaunch(const QString &packageName, const QString &launchName);
  static bool compareFilesByTimestamp(const QString &file1, const QString &file2);

public slots:
  void spinOnce();
  void updateDateTimeGui();
  void updateSuggestions(const QString &text);
  void onItemClicked(QListWidgetItem *item);
  void loadJsonData(fileNameData& fileJsonName);
  void checkForNewFirmware();

private slots:
  void on_wdgTableShopping_itemClicked(QTableWidgetItem *item);

  void on_btSearch_clicked();

  void on_btClear_clicked();

  void on_btnDecreaseVelocity_clicked();

  void on_btnIncreaseVelocity_clicked();

  void on_btnStopResume_clicked();

  void on_btCheckoutCounter_clicked();

protected:
#if USE_MAP_RVIZ

#else
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  bool event(QEvent *event) override;  // Override event function to handle gestures
#endif // USE_MAP_RVIZ

private:
  Ui::robot_gui *ui;
  QTimer *ros_timer;
  QTimer *time_date_gui;
  QListWidget *listWidget;

  QString fwFolderPath;
  qint64 lastTimestampFW;
  QTimer *fwCheckTimer;

 #if USE_MAP_RVIZ
  rviz::RenderPanel *render_panel;
  rviz::VisualizationManager *viz_manager;
#elif !USE_MAP_RVIZ
  ros::Subscriber map_sub;
  ros::Subscriber destinations_sub;
  ros::Subscriber ga_optimize_path_sub;
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void destinationsCallback(const visualization_msgs::Marker::ConstPtr& msg);
  void gaOptimizePathCallback(const nav_msgs::Path::ConstPtr& msg);
  void updateMapDisplay();
  bool gestureEvent(QGestureEvent *event);
  void pinchTriggered(QPinchGesture *gesture);
  QImage mapImage;
  QPoint lastMousePos;
  QPoint mapOffset = QPoint(0, 0);
  double mapScaleFactor = 1.0;  // Default zoom level
  QPixmap mapPixmap;  // Stores the displayed map
  QVector<QPointF>  destinationPoints;
  QVector<QPointF>  gaPathPoints;
  float mapResolution;
#endif //USE_MAP_RIVZ

  ros::NodeHandlePtr nh_;
  ros::Subscriber finish_flag_sub;
  ros::Subscriber robot_vel_sub;
  ros::ServiceClient search_optimize_path_client;
  ros::Subscriber footprintSub;
  ros::Subscriber amcl_pose_sub;
  ros::ServiceClient set_param_client;
  ros::Publisher stop_robot_pub;
  ros::Publisher resume_robot_pub;

  ros::ServiceClient update_FW_client;

  QVector<QPointF> footprintPoints;
  QVector<QString> choosenProductName;
  QVector<fileNameData> fileJsonData;
  QString locationRobot;

  uint32_t gaResultIndex;
  uint32_t rowCount;
  uint32_t columnCount;
  double maxVelocity;
};

#endif // ROBOT_GUI_H

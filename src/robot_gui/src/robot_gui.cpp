#include "robot_gui.h"
#include "loadingdialog.h"
#include "updatefwloading.h"
#include "../../../build/robot_gui/ui_robot_gui.h"
#include <QIcon>
#include <filesystem>
#include <fstream>
#include <string>

using json = nlohmann::json;
namespace fs = std::filesystem;

robot_gui::robot_gui(QWidget *parent) :
  QWidget(parent), rowCount(50), columnCount(3),
  ui(new Ui::robot_gui)
{
  ui->setupUi(this);
  QIcon icon(":/Icons/Search_Icon.png");
  ui->btSearch->setIcon(icon);

  QIcon icon2(":/Icons/Checkout_Counter.png");
  ui->btCheckoutCounter->setIcon(icon2);

  QIcon icon3(":/Icons/Clear.png");
  ui->btClear->setIcon(icon3);

  nh_.reset(new ros::NodeHandle("~"));

  if (nh_->getParam("/move_base/HybridPlannerROS/max_vel_trans", maxVelocity))
  {
      ui->lblMaxVelocity->setText(QString::number(maxVelocity) + " m/s");
  }

  // setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

  // setup the timer that will update time and date
  time_date_gui = new QTimer(this);
  connect(time_date_gui, SIGNAL(timeout()), this, SLOT(updateDateTimeGui()));
  time_date_gui->start(1000);

  // setup table widget shopping
  ui->wdgTableShopping->setRowCount(rowCount);
  ui->wdgTableShopping->setColumnCount(columnCount);

  // Set column names
  QStringList headers = {"Index", "List of products", "Choosen products"};
  ui->wdgTableShopping->setHorizontalHeaderLabels(headers);
  ui->wdgTableShopping->horizontalHeader()->setStyleSheet("font-weight: bold; color: blue;");
  ui->wdgTableShopping->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);
  ui->wdgTableShopping->verticalHeader()->setVisible(false);

  // Disable automatic resizing
  ui->wdgTableShopping->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);

  ui->wdgTableShopping->setColumnWidth(index, 69);
  ui->wdgTableShopping->setColumnWidth(listOfProducts, 187);
  ui->wdgTableShopping->setColumnWidth(choosenProducts, 187);

  // Force UI update
  ui->wdgTableShopping->update();
  ui->wdgTableShopping->repaint();

  qDebug() << "Index: " << ui->wdgTableShopping->columnWidth(index);
  qDebug() << "listOfProducts: " << ui->wdgTableShopping->columnWidth(listOfProducts);
  qDebug() << "choosenProducts: " << ui->wdgTableShopping->columnWidth(choosenProducts);

  loadProductName();

  listWidget = new QListWidget(this);
  listWidget->hide();

  ui->lblLinearVelocity->setText("0m/s");
  ui->lblAngularVelocity->setText("0rad/s");

  // Listen for typing in QLineEdit
  connect(ui->lSearchProduct, &QLineEdit::textChanged, this, &robot_gui::updateSuggestions);
  connect(listWidget, &QListWidget::itemClicked, this, &robot_gui::onItemClicked);

  fwFolderPath = "/home/lamanhvu/uploads"; // Change to your folder path
  lastTimestampFW = 0; // Initialize timestamp tracker

//  fwCheckTimer = new QTimer(this);
//  connect(fwCheckTimer, &QTimer::timeout, this, &robot_gui::checkForNewFirmware);
//  fwCheckTimer->start(10000); // Check every 5 minutes

 #if USE_MAP_RVIZ
  // Create RViz rendering panel
  render_panel = new rviz::RenderPanel();
  render_panel->setParent(this);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(render_panel);
  ui->wdgMap->setLayout(layout);

  // Create Visualization Manager
  viz_manager = new rviz::VisualizationManager(render_panel);
  render_panel->initialize(viz_manager->getSceneManager(), viz_manager);
  viz_manager->initialize();
  viz_manager->startUpdate();

  // Add a grid display
//  rviz::Display *grid = viz_manager->createDisplay("rviz/Grid", "Grid", true);
//  if (grid) {
//    rviz::Property* gridPropertyLines = grid->subProp("Line Style");
//    if (gridPropertyLines)
//        gridPropertyLines->setValue("Lines");

//    rviz::Property* gridPropertyCell = grid->subProp("Cell Size");
//    if (gridPropertyCell)
//        gridPropertyCell->setValue(1.0);
//  }

  // Add a map display from /map topic
  rviz::Display *map = viz_manager->createDisplay("rviz/Map", "Map", true);
  if (map) {
      rviz::Property* topicProperty = map->subProp("Topic");
      if (topicProperty)
          topicProperty->setValue("/map");
  }

  rviz::Display *path = viz_manager->createDisplay("rviz/Path", "Path", true);
  if (path) {
      rviz::Property* pathTopicProperty = path->subProp("Topic");
      if (pathTopicProperty)
          pathTopicProperty->setValue("/global_path/ga_path");  // your path topic name

      // Optional: Set line color and other properties
      rviz::Property* colorProperty = path->subProp("Color");
      if (colorProperty)
          colorProperty->setValue(QColor(0, 255, 0));  // Green

      rviz::Property* lineWidthProperty = path->subProp("Line Width");
      if (lineWidthProperty)
          lineWidthProperty->setValue(0.2);
  }

  rviz::Display *destinationsDisplay = viz_manager->createDisplay("rviz/Marker", "Goods", true);
  destinationsDisplay->subProp("Marker Topic")->setValue("/global_path/destinations_point");

  rviz::Display *goodsDisplay = viz_manager->createDisplay("rviz/Marker", "Goods", true);
  goodsDisplay->subProp("Marker Topic")->setValue("/global_path/goods_point");

  rviz::Display *clustersDisplay = viz_manager->createDisplay("rviz/Marker", "Clusters", true);
  clustersDisplay->subProp("Marker Topic")->setValue("/global_path/clusters_point");

  rviz::Display *footprintPoly = viz_manager->createDisplay("rviz/Polygon", "Footprint", true);
  if (footprintPoly) {
      rviz::Property* topicProperty = footprintPoly->subProp("Topic");
      if (topicProperty)
          topicProperty->setValue("/move_base/local_costmap/footprint");
      rviz::Property* colorProperty = footprintPoly->subProp("Color");
      if (colorProperty)
          colorProperty->setValue(QColor(0, 0, 0)); // black
      rviz::Property* lineWidthProperty = footprintPoly->subProp("Line Width");
      if (lineWidthProperty)
          lineWidthProperty->setValue(0.2);
  }
#elif !USE_MAP_RVIZ
  // Subscribe to /map topic
  map_sub = nh_->subscribe("/map", 1, &robot_gui::mapCallback, this);
  destinations_sub = nh_->subscribe("/global_path/destinations_point", 1000, &robot_gui::destinationsCallback, this);
  ga_optimize_path_sub = nh_->subscribe("/global_path/ga_path", 1000, &robot_gui::gaOptimizePathCallback, this);
  footprintSub = nh_->subscribe("/move_base/local_costmap/footprint", 1, &robot_gui::footprint_callback, this);
  grabGesture(Qt::PinchGesture);
#endif // USE_MAP_RVIZ

  finish_flag_sub = nh_->subscribe("/move_base/HybridPlannerROS/finish_flag", 10, &robot_gui::finish_flag_callback, this);
  robot_vel_sub = nh_->subscribe("/cmd_vel", 10, &robot_gui::robot_velocity_callback, this);
  search_optimize_path_client = nh_->serviceClient<robot_navigation::GARequest>("/GUI_search_optimize");
  amcl_pose_sub = nh_->subscribe("/amcl_pose", 1, &robot_gui::amcl_pose_callback, this);
  set_param_client = nh_->serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/HybridPlannerROS/set_parameters");
  stop_robot_pub = nh_->advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
  resume_robot_pub = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
  real_path_pub = nh_->advertise<nav_msgs::Path>("/global_path/real_path", 1);
  publish_timer = nh_->createTimer(ros::Duration(0.1), &robot_gui::publishRealPath, this);
}

robot_gui::~robot_gui()
{
  delete ui;
  delete ros_timer;
  delete time_date_gui;
}

void robot_gui::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}

void robot_gui::updateDateTimeGui(){
  // Get the current date and time
  QDateTime currentDateTime = QDateTime::currentDateTime();

  // Extract date and time separately
  QDate date = currentDateTime.date();
  QTime time = currentDateTime.time();

  ui->lblTime->setText(QString(time.toString("HH:mm:ss")));
  ui->lblDay->setText(QString(date.toString("yyyy-MM-dd")));
}

void robot_gui::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    PathPoint point;
    point.x = msg->pose.pose.position.x;
    point.y = msg->pose.pose.position.y;

    real_path.push_back(point);
    double minDistance = 1e6;
    for(uint32_t index = 0; index < fileJsonData.size(); index++)
    {
        double distance = hypot(msg->pose.pose.position.x - fileJsonData[index].position[0],
                                msg->pose.pose.position.y - fileJsonData[index].position[1]);
        if(distance <= minDistance)
        {
            minDistance = distance;
            locationRobot = fileJsonData[index].fileName;
        }
    }
    if(minDistance > 1.0)
    {
        locationRobot = "Path";
    }

    ui->lblLocation->setText(locationRobot);
    return;
}

void robot_gui::publishRealPath(const ros::TimerEvent&) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    for (const auto& point : real_path) {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;  // Default orientation

        path.poses.push_back(pose);
    }

    real_path_pub.publish(path);
}

void robot_gui::loadJsonData(fileNameData& fileJsonName)
{
    std::ifstream file(fileJsonName.filePath); // Open the JSON file
    if (!file) {
        std::cerr << "Error: Cannot open file!" << std::endl;
        return;
    }

    json j;
    file >> j; // Parse JSON data

    // Extract position and orientation
    QVector<double> position = j["positon"]; // Note: Key is "positon" (typo in JSON)
    QVector<double> orientation = j["orientation"];

    fileJsonName.position = position;
    fileJsonName.orientation = orientation;
}

void robot_gui::loadProductName(){
  std::string folderPath = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/ProductPose";
  int32_t countProduct = 0;
  fileJsonData.clear();
  try {
      for (const auto& entry : fs::directory_iterator(folderPath)) {
          if (fs::is_regular_file(entry.path())) {
              fileNameData data;
              // Get the filename and convert it to string
              std::string fileName = entry.path().filename().string();

              // Remove the ".json" extension if present
              if (fileName.length() > 5 && fileName.substr(fileName.length() - 5) == ".json") {
                  fileName = fileName.substr(0, fileName.length() - 5);
              }

              // Store product name to a vector for future process GA algorithm
              data.fileName = QString::fromUtf8(fileName.data(), int(fileName.size()));
              data.filePath = entry.path();
              fileJsonData.push_back(data);

              for (uint32_t i = 0; i < fileJsonData.size(); i++)
              {
                  loadJsonData(fileJsonData[i]);
              }

              // Skip if product name is "Start" and "End"
              if((fileName.compare("Start") == 0) || (fileName.compare("CheckoutCounter") == 0))
              {
                  continue;
              }
              ui->wdgTableShopping->setItem(countProduct, TableRowProduct::listOfProducts,
                                            new QTableWidgetItem(QString::fromUtf8(fileName.data(), int(fileName.size()))));
              QTableWidgetItem *indexItem = new QTableWidgetItem(QString::number(++countProduct));
              indexItem->setTextAlignment(Qt::AlignCenter);
              ui->wdgTableShopping->setItem(countProduct - 1, TableRowProduct::index, indexItem);
//              qDebug() << "Product: " << QString::fromUtf8(fileName.data(), int(fileName.size()));
//              qDebug() << "Count: " << countProduct;
          }
      }
  } catch (const std::filesystem::filesystem_error& e) {
      std::cerr << "Filesystem error: " << e.what() << std::endl;
  }

  ui->wdgTableShopping->show();
}

void robot_gui::on_wdgTableShopping_itemClicked(QTableWidgetItem *item)
{
    QString itemText = item->text();
    int column = item->column();
    int row = item->row();
    int rowCount = ui->wdgTableShopping->rowCount();

    switch (column) {
    case TableRowProduct::listOfProducts:
      if(choosenProductName.indexOf(itemText) == -1)
      {
        choosenProductName.push_back(itemText);
        ui->wdgTableShopping->setItem(choosenProductName.count() - 1, TableRowProduct::choosenProducts,
                                      new QTableWidgetItem(itemText));
      }
      break;
    case TableRowProduct::choosenProducts:
      for(int32_t index = row; index < rowCount; index++)
      {
        QTableWidgetItem* belowItem = ui->wdgTableShopping->takeItem(index + 1, column);
        if (belowItem) {
            ui->wdgTableShopping->setItem(index, column, belowItem);  // Move item up
        } else {
            ui->wdgTableShopping->setItem(index, column, new QTableWidgetItem("")); // Fill with empty item
        }
      }
      choosenProductName.removeAt(choosenProductName.indexOf(itemText));
      break;
    default:
      break;
    }
}

void robot_gui::updateSuggestions(const QString &text) {
    listWidget->clear();
    if (text.isEmpty()) {
        listWidget->hide();
        return;
    }

    for(uint32_t index = 0; index < fileJsonData.size(); index++)
    {
      // Skip if product name is "Start" and "End"
        if((fileJsonData[index].fileName.compare("Start") == 0) || (fileJsonData[index].fileName.compare("CheckoutCounter") == 0))
        {
            continue;
        }

        if(fileJsonData[index].fileName.contains(text, Qt::CaseInsensitive)) {
            listWidget->addItem(fileJsonData[index].fileName);
        }
    }

    if (listWidget->count() > 0) {
        listWidget->setGeometry(ui->lSearchProduct->geometry().x()
                                , ui->lSearchProduct->geometry().y() + ui->lSearchProduct->height()
                                , ui->lSearchProduct->width(), 100);
        listWidget->show();
    } else {
        listWidget->hide();
    }
}

void robot_gui::onItemClicked(QListWidgetItem *item) {
  QString itemText = item->text();
  ui->lSearchProduct->setText(itemText);
  if(choosenProductName.indexOf(itemText) == -1)
  {
    choosenProductName.push_back(itemText);
    ui->wdgTableShopping->setItem(choosenProductName.count() - 1, TableRowProduct::choosenProducts,
                                  new QTableWidgetItem(itemText));
  }
  listWidget->hide();
}

void robot_gui::on_btSearch_clicked()
{
  ROS_INFO("Search optimize path");
  real_path.clear();

  LoadingDialog *loading = new LoadingDialog(this);
  loading->show();
  QFuture<void> future = QtConcurrent::run([=]()
  {
    robot_navigation::GARequest srv;
    srv.request.start = locationRobot.toStdString();
    for (auto product : choosenProductName)
    {
        srv.request.destinations.push_back(product.toStdString());
    }

    if (!search_optimize_path_client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Service not available");
        return;
    }

    if (search_optimize_path_client.call(srv))
    {
        choosenProductName.clear();
        fullChoosenProductName.clear();
        fullChoosenProductIndice.clear();
        for (const auto &product : srv.response.GA_result) {
            choosenProductName.push_back(QString::fromUtf8(product.data(), int(product.size())));
        }
        for (const auto &product : srv.response.Products) {
            fullChoosenProductName.push_back(QString::fromUtf8(product.data(), int(product.size())));
        }
        for (const auto &indice : srv.response.Products_indices) {
            fullChoosenProductIndice.push_back(indice);
        }
        QMetaObject::invokeMethod(this, [=]() {
            for(int32_t idx = 0; idx < fullChoosenProductName.size(); idx++)
            {
                ui->wdgTableShopping->setItem(idx, TableRowProduct::choosenProducts,
                                            new QTableWidgetItem(fullChoosenProductName[idx]));
            }
            gaResultIndex = 0;
            for(int32_t idx = 0; idx < fullChoosenProductIndice[gaResultIndex+1]; idx++){
                QTableWidgetItem *item = new QTableWidgetItem(fullChoosenProductName[idx]);
                item->setForeground(QBrush(Qt::blue));
                ui->wdgTableShopping->setItem(idx, TableRowProduct::choosenProducts, item);
            }
        }, Qt::QueuedConnection);
    }
    else
    {
        ROS_ERROR("Failed to call GA service");
    }

    if (!loading->isClosed()) {
        loading->userNotForceClose();
        QMetaObject::invokeMethod(loading, "close", Qt::QueuedConnection);
    }
  });
}

void robot_gui::finish_flag_callback(const std_msgs::Bool::ConstPtr &msg) {
    if(msg->data == true) {
        if(gaResultIndex < fullChoosenProductIndice.size() - 1){
            for(int32_t idx = fullChoosenProductIndice[gaResultIndex]; idx < fullChoosenProductIndice[gaResultIndex+1]; idx++){
                QTableWidgetItem *item = new QTableWidgetItem(fullChoosenProductName[idx]);
                item->setForeground(QBrush(Qt::red));
                ui->wdgTableShopping->setItem(idx, TableRowProduct::choosenProducts, item);
            }
        }

        gaResultIndex++;
        if(gaResultIndex >= choosenProductName.size())
        {
          if(ui->lblLocation->text() == "CheckoutCounter"){
              ui->btnStopResume->setText("RETURN START");
              choosenProductName.push_back("Start");
          }
          else if(ui->lblLocation->text() == "Start"){
              choosenProductName.clear();
              fullChoosenProductName.clear();
              fullChoosenProductIndice.clear();
              for (int row = 0; row < rowCount; ++row)
              {
                  delete ui->wdgTableShopping->takeItem(row, TableRowProduct::choosenProducts);
              }
          }
          return;
        }
        for(int32_t idx = fullChoosenProductIndice[gaResultIndex]; idx < fullChoosenProductIndice[gaResultIndex+1]; idx++){
            QTableWidgetItem *item = new QTableWidgetItem(fullChoosenProductName[idx]);
            item->setForeground(QBrush(Qt::blue));
            ui->wdgTableShopping->setItem(idx, TableRowProduct::choosenProducts, item);
        }
        ui->btnStopResume->setText("NEXT GOODS");
        actionlib_msgs::GoalID cancel_msg;
        stop_robot_pub.publish(cancel_msg);
    }
}

void robot_gui::robot_velocity_callback(const geometry_msgs::Twist::ConstPtr &msg) {
  ui->lblLinearVelocity->setText(QString::number(msg->linear.x) + "m/s");
  ui->lblAngularVelocity->setText(QString::number(msg->angular.z) + "rad/s");
}

#if !USE_MAP_RVIZ
void robot_gui::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    int width = msg->info.width;
    int height = msg->info.height;
    mapResolution = msg->info.resolution;

    mapImage = QImage(width, height, QImage::Format_RGB32);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int index = x + y * width;
            int value = msg->data[index];

            QColor color = (value == 0) ? Qt::white : (value == 100) ? Qt::black : Qt::gray;
            mapImage.setPixel(x, y, color.rgb());
        }
    }

    updateMapDisplay();
}

void robot_gui::updateMapDisplay(void)
{
    if (!mapImage.isNull())
    {
        mapPixmap = QPixmap(ui->wdgMap->size());
        mapPixmap.fill(Qt::gray);  // Set background color

        QPainter painter(&mapPixmap);
        painter.setRenderHint(QPainter::Antialiasing);

        // Scale the map image without flipping
        QSize scaledSize = mapImage.size() * mapScaleFactor;
        QImage scaledImage = mapImage.scaled(scaledSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);

        QPoint drawPosition = mapOffset + QPoint(
            (ui->wdgMap->width() - scaledImage.width()) / 2,
            (ui->wdgMap->height() - scaledImage.height()) / 2
        );
        painter.drawImage(drawPosition, scaledImage);

        QPoint mapCenter = drawPosition + QPoint(scaledSize.width() / 2, scaledSize.height() / 2);

        if (!footprintPoints.isEmpty())
        {
            QPen pen(Qt::red);
            pen.setWidth(2);
            painter.setPen(pen);

            QVector<QPointF> transformedFootprint;

            for (const auto &point : footprintPoints)
            {
                double x = mapCenter.x() + (point.x() / mapResolution + 6) * mapScaleFactor;
                double y = mapCenter.y() + (point.y() / mapResolution + 7) * mapScaleFactor;

                transformedFootprint.push_back(QPointF(x, y));
            }

            // Use drawPolygon to correctly draw footprint
            if (transformedFootprint.size() > 2)
            {
                painter.drawPolygon(transformedFootprint);
            }
            else
            {
                for (int i = 0; i < transformedFootprint.size() - 1; ++i)
                {
                    painter.drawLine(transformedFootprint[i], transformedFootprint[i + 1]);
                }
                painter.drawLine(transformedFootprint.last(), transformedFootprint.first());
            }
        }

        if (!destinationPoints.isEmpty()) {
            QPen pen(Qt::blue);
            pen.setWidth(5);
            painter.setPen(pen);

            for (const auto& point : destinationPoints) {
                double x = mapCenter.x() + (point.x() / mapResolution + 6) * mapScaleFactor;
                double y = mapCenter.y() + (point.y() / mapResolution + 7) * mapScaleFactor;
                painter.drawEllipse(QPointF(x,y), 1, 1);  // Draw points as small circles
            }
        }

        if (!gaPathPoints.isEmpty())
        {
            QPen pen(Qt::green);
            pen.setWidth(1);
            painter.setPen(pen);

            QVector<QPointF> transformedPath;

            for (const auto &point : gaPathPoints)
            {
                double x = mapCenter.x() + (point.x() / mapResolution + 6) * mapScaleFactor;
                double y = mapCenter.y() + (point.y() / mapResolution + 7) * mapScaleFactor;
                transformedPath.push_back(QPointF(x, y));
            }

            for (int i = 0; i < transformedPath.size() - 1; ++i)
            {
                painter.drawLine(transformedPath[i], transformedPath[i + 1]);
            }
        }

        // Force UI update
        ui->wdgMap->update();

        // Apply the updated mapPixmap as background of QWidget
        QPalette palette;
        palette.setBrush(ui->wdgMap->backgroundRole(), QBrush(mapPixmap));
        ui->wdgMap->setPalette(palette);
        ui->wdgMap->setAutoFillBackground(true);
    }
}

void robot_gui::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        lastMousePos = event->pos();
    }
}

void robot_gui::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton)
    {
        QPoint delta = event->pos() - lastMousePos;
        mapOffset += delta;  // Update map position
        lastMousePos = event->pos();  // Store last position

        updateMapDisplay();  // Update the display
    }
}

void robot_gui::wheelEvent(QWheelEvent *event)
{
    double scaleFactor = 1.1;
    if (event->angleDelta().y() > 0)
        mapScaleFactor *= scaleFactor;  // Zoom in
    else
        mapScaleFactor /= scaleFactor;  // Zoom out

    updateMapDisplay();
}

bool robot_gui::event(QEvent *event)
{
    if (event->type() == QEvent::Gesture)
        return gestureEvent(static_cast<QGestureEvent*>(event));
    return QWidget::event(event);
}

bool robot_gui::gestureEvent(QGestureEvent *event)
{
    if (QGesture *gesture = event->gesture(Qt::PinchGesture))
        pinchTriggered(static_cast<QPinchGesture*>(gesture));
    return true;
}

void robot_gui::pinchTriggered(QPinchGesture *gesture)
{
    QPinchGesture::ChangeFlags changeFlags = gesture->changeFlags();

    if (changeFlags & QPinchGesture::ScaleFactorChanged)
    {
        // Adjust zoom level based on pinch scale
        mapScaleFactor *= gesture->scaleFactor();
        updateMapDisplay();
    }
}

void robot_gui::footprint_callback(const geometry_msgs::PolygonStamped::ConstPtr &msg)
{
    footprintPoints.clear();
\
    for (const auto &point : msg->polygon.points)
    {
//        footprintPoints.push_back(QPointF((point.x / mapResolution), (point.y / mapResolution)));
      footprintPoints.push_back(QPointF(point.x, point.y));
    }
    updateMapDisplay();
}

void robot_gui::destinationsCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    destinationPoints.clear();
    for (const auto& point : msg->points) {
        destinationPoints.push_back(QPointF(point.x, point.y));
    }
    updateMapDisplay();
}

void robot_gui::gaOptimizePathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    gaPathPoints.clear();
    for (const auto& pose: msg->poses) {
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        gaPathPoints.push_back(QPointF(x, y));
    }
    qDebug() << "Received Path with" << gaPathPoints.size() << "points";
    updateMapDisplay();
}
#endif // USE_MAP_RVIZ

void robot_gui::on_btClear_clicked()
{
    choosenProductName.clear();
    for (int row = 0; row < rowCount; ++row)
    {
        delete ui->wdgTableShopping->takeItem(row, TableRowProduct::choosenProducts);
    }
}

void robot_gui::on_btnDecreaseVelocity_clicked()
{
    maxVelocity -= 0.05;
    nh_->setParam("/move_base/HybridPlannerROS/max_vel_trans", maxVelocity);
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::DoubleParameter param;
    param.name = "max_vel_trans";
    param.value = maxVelocity;
    srv.request.config.doubles.push_back(param);
    param.name = "max_vel_x";
    param.value = maxVelocity;
    srv.request.config.doubles.push_back(param);

    if (set_param_client.call(srv)) {
        ROS_INFO("Updated successful");
    } else {
        ROS_ERROR("Failed to update");
    }

    if (nh_->getParam("/move_base/HybridPlannerROS/max_vel_trans", maxVelocity))
    {
        ui->lblMaxVelocity->setText(QString::number(maxVelocity) + " m/s");
    }
}

void robot_gui::on_btnIncreaseVelocity_clicked()
{
    maxVelocity += 0.05;
    nh_->setParam("/move_base/HybridPlannerROS/max_vel_trans", maxVelocity);
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::DoubleParameter param;
    param.name = "max_vel_trans";
    param.value = maxVelocity;
    srv.request.config.doubles.push_back(param);
    param.name = "max_vel_x";
    param.value = maxVelocity;
    srv.request.config.doubles.push_back(param);

    if (set_param_client.call(srv)) {
        ROS_INFO("Updated successful");
    } else {
        ROS_ERROR("Failed to update");
    }

    if (nh_->getParam("/move_base/HybridPlannerROS/max_vel_trans", maxVelocity))
    {
        ui->lblMaxVelocity->setText(QString::number(maxVelocity) + " m/s");
    }
}

void robot_gui::on_btnStopResume_clicked()
{
    if (ui->btnStopResume->text() == "STOP")
    {
        ui->btnStopResume->setText("RESUME");
        actionlib_msgs::GoalID cancel_msg;
        stop_robot_pub.publish(cancel_msg);
    }
    else
    {
        ui->btnStopResume->setText("STOP");
        std::string filePath = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/ProductPose/"
            + choosenProductName[gaResultIndex].toStdString() + ".json";
        std::ifstream fileJSON(filePath); // Open the JSON file
        if (!fileJSON) {
            std::cerr << "Error: Cannot open file!" << std::endl;
            return;
        }

        json j;
        fileJSON >> j; // Parse JSON data

        // Extract position and orientation
        std::vector<double> position = j["positon"]; // Note: Key is "positon" (typo in JSON)
        std::vector<double> orientation = j["orientation"];

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = position[0];
        pose.pose.position.y = position[1];
        pose.pose.position.z = position[2];
        pose.pose.orientation.x = orientation[0];
        pose.pose.orientation.y = orientation[1];
        pose.pose.orientation.z = orientation[2];
        pose.pose.orientation.w = orientation[3];

        resume_robot_pub.publish(pose);
    }
}

void robot_gui::on_btCheckoutCounter_clicked()
{
  choosenProductName.push_back("CheckoutCounter");
  ui->wdgTableShopping->setItem(choosenProductName.count() - 1, TableRowProduct::choosenProducts,
                                new QTableWidgetItem("CheckoutCounter"));
}

bool robot_gui::compareFilesByTimestamp(const QString &file1, const QString &file2) {
    // Extract the numeric timestamp from the filename
    qint64 timestamp1 = file1.split("-").first().toLongLong();
    qint64 timestamp2 = file2.split("-").first().toLongLong();

    return timestamp1 < timestamp2;  // Sort in ascending order
}

void robot_gui::checkForNewFirmware()
{
    if (fwFolderPath.isEmpty())
    {
        qDebug() << "Folder path is not set!";
        return;
    }

    QDir dir(fwFolderPath);
    if (!dir.exists())
    {
        qDebug() << "Folder does not exist!";
        return;
    }

    // Ensure oldVersion folder exists
    QString oldVersionPath = fwFolderPath + "/oldVersion";
    QDir oldVersionDir(oldVersionPath);
    if (!oldVersionDir.exists())
    {
        dir.mkdir("oldVersion");
    }

    // Get list of files
    QStringList fileList = dir.entryList(QDir::Files, QDir::Time | QDir::Reversed);

    std::sort(fileList.begin(), fileList.end(), robot_gui::compareFilesByTimestamp);
    qDebug() << "Sorted file list:";
    for (const QString &file : fileList) {
        qDebug() << file;
    }

    // Keep the latest file, move older ones
    for (int i = 0; i < fileList.size() - 1; ++i) {
        QString oldFilePath = fwFolderPath + "/" + fileList[i];
        QString newFilePath = oldVersionPath + "/" + fileList[i];

        if (QFile::rename(oldFilePath, newFilePath)) {
            qDebug() << "Moved:" << fileList[i] << "->" << newFilePath;
        } else {
            qDebug() << "Failed to move:" << fileList[i];
        }
    }

    if(fileList.size() >= 1)
    {
        QString newFile = fileList[fileList.size() - 1];
        qDebug() << "New FW: " << newFile;

        updateFWLoading *updateFW = new updateFWLoading(this);
        updateFW->show();
        QFuture<void> future = QtConcurrent::run([=]()
        {
          robot_navigation::GARequest srv;
          srv.request.start = locationRobot.toStdString();
          for (auto product : choosenProductName)
          {
              srv.request.destinations.push_back(product.toStdString());
          }

          if (!search_optimize_path_client.waitForExistence(ros::Duration(5.0))) {
              ROS_ERROR("Service not available");
              return;
          }

          if (search_optimize_path_client.call(srv))
          {
              choosenProductName.clear();
              for (const auto &product : srv.response.GA_result) {
                  choosenProductName.push_back(QString::fromUtf8(product.data(), int(product.size())));
              }
              QMetaObject::invokeMethod(this, [=]() {
                for(int32_t idx = 0; idx < choosenProductName.size(); idx++)
                {
                    ui->wdgTableShopping->setItem(idx, TableRowProduct::choosenProducts,
                                                new QTableWidgetItem(choosenProductName[idx]));
                }
                gaResultIndex = 0;
                QTableWidgetItem *item = new QTableWidgetItem(choosenProductName[gaResultIndex]);
                item->setForeground(QBrush(Qt::blue));
                ui->wdgTableShopping->setItem(gaResultIndex, TableRowProduct::choosenProducts, item);
              }, Qt::QueuedConnection);
          }
          else
          {
              ROS_ERROR("Failed to call GA service");
          }
        });
    }
}

void robot_gui::killRosLaunch(const QString &launchName) {
    QString command = "pkill -f 'roslaunch .* " + launchName + "'";
    QProcess process;
    process.start("bash", QStringList() << "-c" << command);
    process.waitForFinished();

    QString output = process.readAllStandardOutput();
    QString error = process.readAllStandardError();

    if (!output.isEmpty()) {
        qDebug() << "ROS Launch Killed:" << output;
    }
    if (!error.isEmpty()) {
        qDebug() << "Error:" << error;
    }
}

void robot_gui::startRosLaunch(const QString &packageName, const QString &launchName) {
    QString command = "roslaunch " + packageName + " " + launchName;
    QProcess *process = new QProcess(this);
    process->start("bash", QStringList() << "-c" << command);

    if (!process->waitForStarted()) {
        qDebug() << "Failed to start launch:" << launchName;
        return;
    } else {
        qDebug() << "Launch started successfully:" << launchName;
    }

    connect(process, &QProcess::readyReadStandardOutput, [=]() {
        qDebug() << process->readAllStandardOutput();
    });

    connect(process, &QProcess::readyReadStandardError, [=]() {
        qDebug() << "[ERROR] " << process->readAllStandardError();
    });
}

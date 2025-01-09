#ifndef FINDING_PATH_H
#define FINDING_PATH_H

#include "A_star.hpp"

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>


class FindingPath
{
public:
    FindingPath();
    ~FindingPath();

private:
    void subscribeAndPublish();
    void gridMapHandler(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
    void navGoalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal_msg);

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_grid_map_;
    ros::Subscriber sub_nav_goal_;
    ros::Publisher  pub_robot_path_;

    AStar::Generator map_generator_;
    nav_msgs::MapMetaData map_info_;
    bool map_exsit_;
};

#endif //FINDING_PATH_H

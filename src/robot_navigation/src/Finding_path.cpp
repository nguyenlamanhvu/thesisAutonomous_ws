#include "../include/robot_navigation/Finding_path.h"

#include <iostream>

FindingPath::FindingPath()
{
    subscribeAndPublish();
}

FindingPath::~FindingPath()
{

}

void FindingPath::subscribeAndPublish()
{
    sub_grid_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1, &FindingPath::gridMapHandler, this);
    sub_nav_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, &FindingPath::navGoalHandler, this);
    pub_robot_path_ = nh_.advertise<nav_msgs::Path>("/global_path/path", 1, true);
}

void FindingPath::gridMapHandler(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    ROS_INFO("Generating map..");
    map_exsit_ = false;

    map_info_ = map_msg->info;

    // Generate Map, Options
    map_generator_.setWorldSize({(int)map_info_.width, (int)map_info_.height}); //{x, y}
    map_generator_.setHeuristic(AStar::Heuristic::euclidean);
    map_generator_.setDiagonalMovement(true);

    // Add Wall
    int x, y;
    for(int i=0; i<map_info_.width*map_info_.height; i++)
    {
        x = i%map_info_.width;
        y = i/map_info_.width;

        if(map_msg->data[i] != 0)
        {
            map_generator_.addCollision({x, y}, 3);
        }
    }

    ROS_INFO("Success build map!");
    map_exsit_ = true;
}

void FindingPath::navGoalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
{
    if(!map_exsit_) return;

    ROS_INFO("\033[1;32mGoal received!\033[0m");

    // Round goal coordinate
    float goal_x = round(goal_msg->pose.position.x*10)/10;
    float goal_y = round(goal_msg->pose.position.y*10)/10;

    // Remmaping coordinate
    AStar::Vec2i target;
    target.x = (goal_x - map_info_.origin.position.x) / map_info_.resolution;
    target.y = (goal_y - map_info_.origin.position.y) / map_info_.resolution;

    AStar::Vec2i source;
    source.x = (0 - map_info_.origin.position.x) / map_info_.resolution;
    source.y = (0 - map_info_.origin.position.y) / map_info_.resolution;

    // Find Path
    auto path = map_generator_.findPath(source, target);

    nav_msgs::Path path_msg;
    
    if(path.empty())
    {
        ROS_INFO("\033[1;31mFail generate path!\033[0m");
        return;
    }

    path_msg.poses.resize(path.size());
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    for(auto coordinate=path.end()-1; coordinate>=path.begin(); --coordinate)
    {
        geometry_msgs::PoseStamped point_pose;

        // Remmaping coordinate
        point_pose.pose.position.x = (coordinate->x + map_info_.origin.position.x / map_info_.resolution) * map_info_.resolution;
        point_pose.pose.position.y = (coordinate->y + map_info_.origin.position.y / map_info_.resolution) * map_info_.resolution;
        path_msg.poses.push_back(point_pose);
    }


    pub_robot_path_.publish(path_msg);

    ROS_INFO("\033[1;36mSuccess generate path!\033[0m");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "finding_path");

    ROS_INFO("\033[1;32m----> Finding Path Node is Started.\033[0m");

    FindingPath FindPath;

    ros::spin();
    return 0;
}

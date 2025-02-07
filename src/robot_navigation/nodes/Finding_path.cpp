#include "../include/robot_navigation/Finding_path.h"

#include <iostream>

int main(int argc, char **argv) {

	ros::init(argc, argv, "astar");

	ros::NodeHandle nh;

	// Publishers
	start_pose_pub = nh.advertise<geometry_msgs::PointStamped>("start_pose", 1000);
	goal_pose_pub = nh.advertise<geometry_msgs::PointStamped>("goal_pose", 1000);
	astar_path_pub = nh.advertise<nav_msgs::Path>("/global_path/path", 1000);

	// Subscribers
	ros::Subscriber start_pose_sub = nh.subscribe("initialpose", 1000, callback_start_pose);
	ros::Subscriber goal_pose_sub = nh.subscribe("move_base_simple/goal", 1000, callback_goal_pose);
	ros::Subscriber map_sub = nh.subscribe("map", 1, callback_map);
	ros::Subscriber amcl_sub = nh.subscribe("amcl_pose", 1000, callback_amcl_pose);

	// Services
	ros::ServiceServer service = nh.advertiseService("replan_astar", replanAStar);

	ros::spin();

	return 0;
}

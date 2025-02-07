#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <iostream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <queue>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PolygonStamped.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "robot_navigation/ReplanPath.h"

using namespace std;

typedef pair<float, int> pi;

// Publisher Variables
ros::Publisher start_pose_pub;
ros::Publisher goal_pose_pub;
ros::Publisher astar_path_pub;

// Variables for start and goal points
float sx;
float sy;
float gx;
float gy;

geometry_msgs::PointStamped start_point; // Start pose msg
bool valid_start; // Start pose validity check
geometry_msgs::PointStamped goal_point; // Goal pose msg
bool valid_goal; // Goal pose validity check
geometry_msgs::PointStamped amcl_point; // AMCL pose msg

nav_msgs::OccupancyGrid::Ptr grid; // Pointer to the occupancy grid msg
int grid_height;
int grid_width;
int grid_originalX;
int grid_originalY;
bool** bin_map; // 2D Binary map of the grid 
int** acc_obs_map;

nav_msgs::Path path; // Astar Path

struct pathPoint {
	double position_x;
	double position_y;
	double orientation_w;
};

class Node2D {

public:

	Node2D() {};

	Node2D(float x, float y, float theta, float cost, int pind) {

		this->x = x;
		this->y = y;
		this->cost = cost;
		this->g_cost = 0;
		this->pind = pind;
		this->theta = theta;
	}

	float get_x() const { return x; }

	float get_y() const { return y; }

	float get_theta() const { return theta; }

	float get_cost() const { return cost; }

	float get_g_cost() const { return g_cost; }

	int get_pind() const { return pind; }

	void set_g_cost(float g_cost) { this->g_cost = g_cost; }

private:

	float x; // x co-ordinate of the node
	float y; // y co-ordinate of the node
	float theta; // orientation of the node
	float cost; // cost of the node
	float g_cost; // g cost of the node
	int pind; // parent index of the node

};

float astar(float sx, float sy, float gx, float gy, nav_msgs::Path& astar_path);
void callback_start_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
void callback_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& pose);
void callback_map(const nav_msgs::OccupancyGrid::Ptr map);
void callback_amcl_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
bool replanAStar(robot_navigation::ReplanPath::Request &req, 
                 robot_navigation::ReplanPath::Response &res);

#endif // ASTAR_HPP
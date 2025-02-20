#include "../include/robot_navigation/GA.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GA");
    ros::NodeHandle private_nh("~");

    PathPlanningGA ga;

    ROS_INFO("GA service is ready.");
    ros::spin();

    return 0;
}
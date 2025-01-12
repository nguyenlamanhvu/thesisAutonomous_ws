#include "../include/robot_navigation/DWA.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dwa_planner");
  DWAPlanner planner;
  planner.process();
  return 0;
}
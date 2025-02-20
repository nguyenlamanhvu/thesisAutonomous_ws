#ifndef GA_PLANNER_GA_PLANNER_H
#define GA_PLANNER_GA_PLANNER_H

#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <string>
#include <fstream>
#include "json.hpp"
#include <map>
#include <ctime>

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
#include "robot_navigation/GARequest.h"

class PathPlanningGA {
private:
    struct Individual {
        std::vector<int> chromosome;
        double fitness;
    };

    int populationSize;
    int generations;
    double mutationRate;
    double crossoverRate;
    std::vector<Individual> population;
    std::map<std::pair<std::string, std::string>, double> costMap;
    std::vector<std::string> locationNames;
    std::string startLocation;

    // Convert location names to indices for easier manipulation
    std::map<std::string, int> locationToIndex;
    std::map<int, std::string> indexToLocation;
    ros::ServiceServer GA_planning_server;

public:
    PathPlanningGA(void);

    void loadCostData(const std::string& start, const std::vector<std::string>& destinations);

    void initializePopulation(const std::string& start, const std::vector<std::string>& destinations);

    double calculateFitness(const std::vector<int>& chromosome);

    std::vector<int> crossover(const std::vector<int>& parent1, const std::vector<int>& parent2);

    void mutate(std::vector<int>& chromosome);

    std::vector<std::string> optimize();

private:
    Individual tournament(int tournamentSize = 5);

    double getCostFromJson(const std::string& from, const std::string& to);

    bool handleGARequest(robot_navigation::GARequest::Request &req,
                   robot_navigation::GARequest::Response &res);
};

#endif /*GA_PLANNER_GA_PLANNER_H*/
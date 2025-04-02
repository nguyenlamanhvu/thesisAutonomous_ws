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
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>

#include <ros/ros.h>
#include <ros/callback_queue.h>
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
#include "std_msgs/Bool.h"
#include "robot_navigation/GARequest.h"

// Structure to represent a good in the supermarket
struct Good {
    double x, y;
    std::string name;
    Good(double x = 0, double y = 0, const std::string& name = "") : x(x), y(y), name(name) {}
};

double distance(const Good& p1, const Good& p2);

class KMeans {
private:
    int k; // Number of clusters
    std::vector<Good> goods; // Input goods
    std::vector<Good> centroids; // Cluster centroids
    std::vector<int> assignments; // Cluster assignments for each good
    double inertia; // Sum of squared distances to nearest centroid
    double maxRadius; // Maximum allowed radius for each cluster
    
public:
KMeans(int k, double radius = std::numeric_limits<double>::max()) : k(k), inertia(0.0), maxRadius(radius) {}
    
    // Add a good to the dataset
    void addGood(const Good& g);
    
    // Get centroid at specified index
    Good getCentroid(int index);
    
    // K-means++ initialization
    void initializeCentroidsPlusPlus();
    
    // Assign goods to nearest centroid
    bool assignClusters();
    
    // Update centroid positions
    void updateCentroids();
    
    double getInertia();
    
    // Run k-means clustering
    void run(int maxIterations = 100);
    
    // Get clustering results as array of arrays with good names
    void printClusterResults();

    // Get all goods in each cluster
    std::vector<std::vector<std::string>> getClusterContents();
};

std::vector<Good> readGoodsFromFiles(const std::string& folderPath);
int findOptimalK(const std::vector<Good>& goods, double maxRadius = std::numeric_limits<double>::max(), int maxK = 100);
std::vector<Good> selectGoods(const std::vector<Good>& allGoods, const std::vector<std::string>& selectedNames);
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
    std::string endLocation;
    bool hasEndLocation = false;

    // Convert location names to indices for easier manipulation
    std::map<std::string, int> locationToIndex;
    std::map<int, std::string> indexToLocation;
    ros::ServiceServer GA_planning_server;
    ros::Subscriber Stop_GA_flag_sub;
    bool stopGA = false;
    ros::CallbackQueue high_priority_queue;
    std::unique_ptr<ros::AsyncSpinner> spinner;

public:
    PathPlanningGA(void);
    ~PathPlanningGA();

    void loadCostData(const std::string& start, const std::vector<std::string>& destinations);

    void initializePopulation(const std::string& start, const std::vector<std::string>& destinations);

    double calculateFitness(const std::vector<int>& chromosome);

    std::vector<int> crossover(const std::vector<int>& parent1, const std::vector<int>& parent2);

    void mutate(std::vector<int>& chromosome);

    std::vector<std::string> optimize();

    void setEndPosition(const std::string& end);

private:
    Individual tournament(int tournamentSize = 5);

    double getCostFromJson(const std::string& from, const std::string& to);

    bool handleGARequest(robot_navigation::GARequest::Request &req,
                   robot_navigation::GARequest::Response &res);

    void stopGAFlag(const std_msgs::Bool::ConstPtr &msg);
};

#endif /*GA_PLANNER_GA_PLANNER_H*/

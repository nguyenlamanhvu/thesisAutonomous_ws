#include "../include/robot_navigation/json.hpp"
#include "../include/robot_navigation/A_star.hpp"
#include "../include/robot_navigation/GA.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <vector>
#include <tuple>
#include <map>
#include "std_msgs/Empty.h"
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

using json = nlohmann::json;
namespace fs = std::filesystem;

class MoveBase {
private:
    struct fileNameData {
        std::string fileName;
        std::string filePath;
        std::vector<double> position;
        std::vector<double> orientation; 
    };

    std::vector<fileNameData> fileJsonData;
    std::vector<std::string> gaResult;
    uint32_t gaResultIndex;

    ros::Subscriber create_astar_result_sub;
    ros::ServiceClient replan_astar_result_client;

    ros::Subscriber ga_planning_sub;
    ros::ServiceClient replan_ga_planning_client;
    ros::ServiceServer ga_gui_server;

    ros::Publisher ga_optimize_path_pub;
    ros::Publisher destinations_pub;

    ros::Subscriber finish_flag_sub;
    ros::Publisher move_base_goal_pub;
    ros::Publisher astar_global_path_pub;

    void create_astar_result_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_INFO_STREAM("Create Astar Result Callback");
        createJsonFile();
    }

    bool handleGAGUIRequest(robot_navigation::GARequest::Request &req,
        robot_navigation::GARequest::Response &res)
    {
        gaResult.clear();
        ROS_INFO("Received GUI request:");

        robot_navigation::GARequest srv;

        srv.request.start = req.start;
        srv.request.destinations = req.destinations;

        if (replan_ga_planning_client.call(srv))
        {
            res.GA_result = srv.response.GA_result;
            gaResult = res.GA_result;
            gaResult.insert(gaResult.begin(), req.start);
        }
        else
        {
            ROS_ERROR("Failed to call GA service");
        }

        nav_msgs::Path ga_optimize_path;
        ga_optimize_path.header.stamp = ros::Time::now();
        ga_optimize_path.header.frame_id = "/map";

        for (uint32_t i = gaResult.size() - 1; i > 0; i--)
        {
            std::string filename = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/AStarResult/" + gaResult[i-1] + "To" + gaResult[i] + ".json";
            ROS_INFO_STREAM("A Star path: " << gaResult[i-1] << " To " << gaResult[i]);
            readAStarPath(filename, ga_optimize_path);
        }

        visualization_msgs::Marker ga_point_path = createWaypointsMarker();

        ga_optimize_path_pub.publish(ga_optimize_path);
        destinations_pub.publish(ga_point_path);

        gaResultIndex = 1;
        publishPathAndGoal();
        return true;
    }

    void ga_planning_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        gaResult.clear();
        ROS_INFO_STREAM("Planning GA optimize");

        robot_navigation::GARequest srv;

        for (auto json : fileJsonData)
        {
            if(strcmp(json.fileName.c_str(), "Start") == 0) continue;
            srv.request.destinations.push_back(json.fileName.c_str());
        }

        srv.request.start = {"Start"};

        if (replan_ga_planning_client.call(srv))
        {
            gaResult = srv.response.GA_result;
            gaResult.insert(gaResult.begin(), "Start");
        }
        else
        {
            ROS_ERROR("Failed to call service");
        }

        nav_msgs::Path ga_optimize_path;
        ga_optimize_path.header.stamp = ros::Time::now();
        ga_optimize_path.header.frame_id = "/map";

        for (uint32_t i = gaResult.size() - 1; i > 0; i--)
        {
            std::string filename = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/AStarResult/" + gaResult[i-1] + "To" + gaResult[i] + ".json";
            ROS_INFO_STREAM("A Star path: " << gaResult[i-1] << " To " << gaResult[i]);
            readAStarPath(filename, ga_optimize_path);
        }

        visualization_msgs::Marker ga_point_path = createWaypointsMarker();

        ga_optimize_path_pub.publish(ga_optimize_path);
        destinations_pub.publish(ga_point_path);

        gaResultIndex = 1;
        publishPathAndGoal();

    }

    void finish_flag_callback(const std_msgs::Bool::ConstPtr &msg) {
        if(msg->data == true) {
            gaResultIndex++;
            if(gaResultIndex >= gaResult.size()) return;
            publishPathAndGoal();
        }
    }

    void publishPathAndGoal(void) {
        // std::string filename = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/AStarResult/" + gaResult[gaResultIndex-1] + "To" + gaResult[gaResultIndex] + ".json";
        // ROS_INFO_STREAM("A Star path: " << gaResult[gaResultIndex-1] << " To " << gaResult[gaResultIndex]);
        // nav_msgs::Path astarPath;
        // astarPath.header.stamp = ros::Time::now();
	    // astarPath.header.frame_id = "map";

        // geometry_msgs::PoseStamped ps;
        // ps.header.stamp = ros::Time::now();
        // ps.header.frame_id = "map";

        // std::ifstream file(filename, std::ifstream::binary);
        // if (!file) {
        //     std::cerr << "Error opening JSON file: " << filename << std::endl;
        //     return;
        // }

        // json root;
        // file >> root;  // Parse JSON into the root object

        // // Extract path
        // const json pathArray = root["path"];
        // for (const auto &point : pathArray) {
        //     ps.pose.position.x = point["x"].get<double>();
        //     ps.pose.position.y = point["y"].get<double>();
        //     ps.pose.orientation.z = point["qw"].get<double>();

        //     astarPath.poses.push_back(ps);
        // }
        // astar_global_path_pub.publish(astarPath);

        std::string filePath = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/ProductPose/" + gaResult[gaResultIndex] + ".json";
        ROS_INFO_STREAM("Goal: " + gaResult[gaResultIndex]);
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

        move_base_goal_pub.publish(pose);
    }

public:
    MoveBase(void) {
        ros::NodeHandle nh;
        create_astar_result_sub = nh.subscribe("Create_AStar_Result", 10, &MoveBase::create_astar_result_callback, this);
        replan_astar_result_client = nh.serviceClient<robot_navigation::ReplanPath>("replan_astar");

        ga_planning_sub = nh.subscribe("GA_Planning", 10, &MoveBase::ga_planning_callback, this);
        replan_ga_planning_client = nh.serviceClient<robot_navigation::GARequest>("GA_optimize");
        ga_gui_server = nh.advertiseService("GUI_search_optimize", &MoveBase::handleGAGUIRequest, this);

        ga_optimize_path_pub = nh.advertise<nav_msgs::Path>("/global_path/ga_path", 1000);
        destinations_pub = nh.advertise<visualization_msgs::Marker>("/global_path/destinations_point", 1000);

        finish_flag_sub = nh.subscribe("/move_base/HybridPlannerROS/finish_flag", 10, &MoveBase::finish_flag_callback, this);
        move_base_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
        astar_global_path_pub = nh.advertise<nav_msgs::Path>("/global_path/path", 1000);
    }

    void loadJsonData(fileNameData& fileJsonName)
    {
        std::ifstream file(fileJsonName.filePath); // Open the JSON file
        if (!file) {
            std::cerr << "Error: Cannot open file!" << std::endl;
            return;
        }

        json j;
        file >> j; // Parse JSON data

        // Extract position and orientation
        std::vector<double> position = j["positon"]; // Note: Key is "positon" (typo in JSON)
        std::vector<double> orientation = j["orientation"];

        fileJsonName.position = position;
        fileJsonName.orientation = orientation;
    }

    void loadPathFile(void)
    {
        fileJsonData.clear();

        std::string folderPath = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/ProductPose";
        try {
            for (const auto& entry : fs::directory_iterator(folderPath)) {
                if (fs::is_regular_file(entry.path())) { 
                    fileNameData data;
                    // Get the filename and convert it to string
                    std::string fileName = entry.path().filename().string();
                    
                    // Remove the ".json" extension if present
                    if (fileName.length() > 5 && fileName.substr(fileName.length() - 5) == ".json") {
                        data.fileName = fileName.substr(0, fileName.length() - 5);
                    }
                    data.filePath = entry.path();

                    fileJsonData.push_back(data);
                }
            }
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "Filesystem error: " << e.what() << std::endl;
        }

        for (uint32_t i = 0; i < fileJsonData.size(); i++)
        {
            loadJsonData(fileJsonData[i]);
        }

        for (auto json : fileJsonData)
        {
            std::cout << "File: " << json.fileName.c_str() << std::endl;
            std::cout << "Position: ";
            for (double p : json.position) {
                std::cout << p << " ";
            }
            std::cout << std::endl;

            std::cout << "Orientation: ";
            for (double o : json.orientation) {
                std::cout << o << " ";
            }
            std::cout << std::endl;
        }
    }

    void createJsonFile(void)
    {
        ROS_INFO_STREAM("Create Json File");

        for(uint32_t i = 0; i < fileJsonData.size(); i++) {
            for(uint32_t j = 0; j < fileJsonData.size(); j++) {
                if (i == j) continue;
                // Create a JSON object
                nlohmann::json json_file;

                // Assign the vectors to the corresponding JSON fields
                json_file["start_position"] = fileJsonData[i].position;
                json_file["goal_position"] = fileJsonData[j].position;

                robot_navigation::ReplanPath srv;

                srv.request.start_pose.pose.position.x = fileJsonData[i].position[0];
                srv.request.start_pose.pose.position.y = fileJsonData[i].position[1];
                srv.request.goal_pose.pose.position.x = fileJsonData[j].position[0];
                srv.request.goal_pose.pose.position.y = fileJsonData[j].position[1];

                if (replan_astar_result_client.call(srv)) {
                    ROS_INFO("Received path with %ld waypoints", srv.response.planned_path.poses.size());
                    json_file["cost"] = srv.response.cost.data;

                    // Store the frame_id
                    json_file["frame_id"] = srv.response.planned_path.header.frame_id;
                    // Store all poses
                    for (const auto& pose : srv.response.planned_path.poses) {
                        json pose_json;
                        pose_json["x"] = pose.pose.position.x;
                        pose_json["y"] = pose.pose.position.y;
                        pose_json["z"] = pose.pose.position.z;

                        pose_json["qx"] = pose.pose.orientation.x;
                        pose_json["qy"] = pose.pose.orientation.y;
                        pose_json["qz"] = pose.pose.orientation.z;
                        pose_json["qw"] = pose.pose.orientation.w;

                        json_file["path"].push_back(pose_json);
                    }
                } else {
                    ROS_ERROR("Failed to call A* service");
                }

                // Specify the file path where you want to save the JSON file
                std::filesystem::path filePath = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/AStarResult/";

                // Manually concatenate the path and file name
                std::string fullFileName = fileJsonData[i].fileName + "To" + fileJsonData[j].fileName + ".json";

                // Append the file name to the path using operator/
                filePath /= fullFileName;

                // Open the file to write the JSON data
                std::ofstream outputFile(filePath);

                // Check if the file was opened successfully
                if (outputFile.is_open()) {
                    // Write the JSON data to the file with pretty print (4 spaces indentation)
                    outputFile << json_file.dump(4);
                    outputFile.close();  // Close the file after writing
                    std::cout << "JSON data has been written to " << filePath << std::endl;
                } else {
                    std::cerr << "Failed to open the file for writing!" << std::endl;
                }
            }
        }
    }

    // Function to read JSON file and parse the A* path
    void readAStarPath(const std::string &filename, nav_msgs::Path& astarPath) {
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time::now();
        ps.header.frame_id = "/map";

        std::ifstream file(filename, std::ifstream::binary);
        if (!file) {
            std::cerr << "Error opening JSON file: " << filename << std::endl;
            return;
        }

        json root;
        file >> root;  // Parse JSON into the root object

        // Extract path
        const json pathArray = root["path"];
        for (const auto &point : pathArray) {
            ps.pose.position.x = point["x"].get<double>();
            ps.pose.position.y = point["y"].get<double>();
            ps.pose.orientation.z = point["qw"].get<double>();

            astarPath.poses.push_back(ps);
        }
    }

    // Function to create markers for waypoints
    visualization_msgs::Marker createWaypointsMarker(void) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "/global_path/destiantion_points";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;  // Point size
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;  // Red color
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        // marker.text = productName;

        for (const auto &productName : gaResult) {
            std::string filePath = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/ProductPose/" + productName + ".json";

            std::ifstream file(filePath); // Open the JSON file
            if (!file) {
                std::cerr << "Error: Cannot open file!" << std::endl;
                continue;
            }

            json j;
            file >> j; // Parse JSON data

            // Extract position and orientation
            std::vector<double> position = j["positon"]; // Note: Key is "positon" (typo in JSON)

            geometry_msgs::Point p;
            p.x = position[0];
            p.y = position[1];
            p.z = position[2];
            marker.points.push_back(p);
        }

        return marker;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_base");
    ros::NodeHandle private_nh("~");

    MoveBase moveBase;

    moveBase.loadPathFile();
    // moveBase.createJsonFile();

    ros::spin();

    return 0;
}

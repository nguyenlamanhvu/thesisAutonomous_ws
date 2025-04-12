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
#include <nav_msgs/GetPlan.h>

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
    std::vector<std::string> goodsResult;
    std::vector<int> goodsIndices;
    uint32_t gaResultIndex;

    ros::Subscriber create_astar_result_sub;
    ros::ServiceClient replan_astar_result_client;
    ros::ServiceClient get_plan_client;

    ros::Subscriber ga_planning_sub;
    ros::ServiceClient replan_ga_planning_client;
    ros::ServiceServer ga_gui_server;

    ros::Publisher ga_optimize_path_pub;
    ros::Publisher destinations_pub;
    ros::Publisher goods_pub;
    ros::Publisher clusters_pub;

    ros::Subscriber finish_flag_sub;
    ros::Publisher move_base_goal_pub;
    ros::Publisher astar_global_path_pub;

    ros::Subscriber Stop_GA_flag_sub;
    bool stopGA = false;
    ros::CallbackQueue high_priority_queue;
    std::unique_ptr<ros::AsyncSpinner> spinner;

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
            res.Products = srv.response.Products;
            res.Products_indices = srv.response.Products_indices;
            gaResult = res.GA_result;
            gaResult.insert(gaResult.begin(), req.start);
            goodsResult = res.Products;
            goodsIndices = res.Products_indices;
            if(stopGA) {
                return true;
            }
        }
        else
        {
            ROS_ERROR("Failed to call GA service");
            return false;
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
        visualization_msgs::Marker good_point_path = createGoodsMarker();

        ga_optimize_path_pub.publish(ga_optimize_path);
        destinations_pub.publish(ga_point_path);
        goods_pub.publish(good_point_path);

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
            return;
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
            if(gaResultIndex < goodsIndices.size()){
                visualization_msgs::Marker cluster_id = createClusterMarker();
                clusters_pub.publish(cluster_id);
            } 
            gaResultIndex++;
            if(gaResultIndex >= gaResult.size()) return;
            publishPathAndGoal();  
        }
    }

    void stopGAFlag(const std_msgs::Bool::ConstPtr &msg) {
        ROS_INFO("Get GA stop flag move_base_integrate");
        stopGA = msg->data;
    }

    void publishPathAndGoal(void) {
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
        ros::NodeHandle private_nh("~");

        private_nh.setCallbackQueue(&high_priority_queue);

        create_astar_result_sub = nh.subscribe("Create_AStar_Result", 10, &MoveBase::create_astar_result_callback, this);
        replan_astar_result_client = nh.serviceClient<robot_navigation::ReplanPath>("replan_astar");
        get_plan_client = nh.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");

        ga_planning_sub = nh.subscribe("GA_Planning", 10, &MoveBase::ga_planning_callback, this);
        replan_ga_planning_client = nh.serviceClient<robot_navigation::GARequest>("GA_optimize");
        ga_gui_server = nh.advertiseService("GUI_search_optimize", &MoveBase::handleGAGUIRequest, this);

        ga_optimize_path_pub = nh.advertise<nav_msgs::Path>("/global_path/ga_path", 1000);
        destinations_pub = nh.advertise<visualization_msgs::Marker>("/global_path/destinations_point", 1000);
        goods_pub = nh.advertise<visualization_msgs::Marker>("/global_path/goods_point", 1000);
        clusters_pub = nh.advertise<visualization_msgs::Marker>("/global_path/clusters_point", 1000);

        finish_flag_sub = nh.subscribe("/move_base/HybridPlannerROS/finish_flag", 10, &MoveBase::finish_flag_callback, this);
        move_base_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
        astar_global_path_pub = nh.advertise<nav_msgs::Path>("/global_path/path", 1000);

        Stop_GA_flag_sub = private_nh.subscribe("/GA_stop_flag", 10, &MoveBase::stopGAFlag, this);

        spinner = std::make_unique<ros::AsyncSpinner>(1, &high_priority_queue);
        spinner->start();
    }
    
    ~MoveBase() {
        if (spinner) {
            spinner->stop();
        }
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

    // void createJsonFile(void)
    // {
    //     ROS_INFO_STREAM("Create Json File");

    //     for(uint32_t i = 0; i < fileJsonData.size(); i++) {
    //         for(uint32_t j = 0; j < fileJsonData.size(); j++) {
    //             if (i == j) continue;
    //             // Create a JSON object
    //             nlohmann::json json_file;

    //             // Assign the vectors to the corresponding JSON fields
    //             json_file["start_position"] = fileJsonData[i].position;
    //             json_file["goal_position"] = fileJsonData[j].position;

    //             robot_navigation::ReplanPath srv;

    //             srv.request.start_pose.pose.position.x = fileJsonData[i].position[0];
    //             srv.request.start_pose.pose.position.y = fileJsonData[i].position[1];
    //             srv.request.goal_pose.pose.position.x = fileJsonData[j].position[0];
    //             srv.request.goal_pose.pose.position.y = fileJsonData[j].position[1];

    //             if (replan_astar_result_client.call(srv)) {
    //                 ROS_INFO("Received path with %ld waypoints", srv.response.planned_path.poses.size());
    //                 json_file["cost"] = srv.response.cost.data;

    //                 // Store the frame_id
    //                 json_file["frame_id"] = srv.response.planned_path.header.frame_id;
    //                 // Store all poses
    //                 for (const auto& pose : srv.response.planned_path.poses) {
    //                     json pose_json;
    //                     pose_json["x"] = pose.pose.position.x;
    //                     pose_json["y"] = pose.pose.position.y;
    //                     pose_json["z"] = pose.pose.position.z;

    //                     pose_json["qx"] = pose.pose.orientation.x;
    //                     pose_json["qy"] = pose.pose.orientation.y;
    //                     pose_json["qz"] = pose.pose.orientation.z;
    //                     pose_json["qw"] = pose.pose.orientation.w;

    //                     json_file["path"].push_back(pose_json);
    //                 }
    //             } else {
    //                 ROS_ERROR("Failed to call A* service");
    //             }

    //             // Specify the file path where you want to save the JSON file
    //             std::filesystem::path filePath = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/AStarResult/";

    //             // Manually concatenate the path and file name
    //             std::string fullFileName = fileJsonData[i].fileName + "To" + fileJsonData[j].fileName + ".json";

    //             // Append the file name to the path using operator/
    //             filePath /= fullFileName;

    //             // Open the file to write the JSON data
    //             std::ofstream outputFile(filePath);

    //             // Check if the file was opened successfully
    //             if (outputFile.is_open()) {
    //                 // Write the JSON data to the file with pretty print (4 spaces indentation)
    //                 outputFile << json_file.dump(4);
    //                 outputFile.close();  // Close the file after writing
    //                 std::cout << "JSON data has been written to " << filePath << std::endl;
    //             } else {
    //                 std::cerr << "Failed to open the file for writing!" << std::endl;
    //             }
    //         }
    //     }
    // }

    double calculatePathLength(const nav_msgs::Path& path) {
        double total_length = 0.0;
    
        // Iterate through the poses in the path
        for (size_t i = 1; i < path.poses.size(); ++i) {
            const auto& p1 = path.poses[i - 1].pose.position;
            const auto& p2 = path.poses[i].pose.position;
    
            // Calculate the Euclidean distance between consecutive points
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double dz = p2.z - p1.z;
    
            total_length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
    
        return total_length;
    }

    bool getPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) {
        nav_msgs::GetPlan srv;
        srv.request.start = start;
        srv.request.goal = goal;
        srv.request.tolerance = 0.1;  // Set a tolerance for the planner
    
        if (get_plan_client.call(srv)) {
            path = srv.response.plan;
            ROS_INFO_STREAM("Received path with " << path.poses.size() << " waypoints.");
            return true;
        } else {
            ROS_ERROR("Failed to call service move_base/make_plan");
            return false;
        }
    }

    void createJsonFile(void) {
        ROS_INFO_STREAM("Create Json File");
    
        for (uint32_t i = 0; i < fileJsonData.size(); i++) {
            for (uint32_t j = 0; j < fileJsonData.size(); j++) {
                if (i == j) continue;
    
                // Create a JSON object
                nlohmann::json json_file;
    
                // Assign the vectors to the corresponding JSON fields
                json_file["start_position"] = fileJsonData[i].position;
                json_file["goal_position"] = fileJsonData[j].position;
    
                // Create start and goal poses
                geometry_msgs::PoseStamped start_pose, goal_pose;
                start_pose.header.frame_id = "map";
                start_pose.pose.position.x = fileJsonData[i].position[0];
                start_pose.pose.position.y = fileJsonData[i].position[1];
                start_pose.pose.orientation.w = 1.0;
    
                goal_pose.header.frame_id = "map";
                goal_pose.pose.position.x = fileJsonData[j].position[0];
                goal_pose.pose.position.y = fileJsonData[j].position[1];
                goal_pose.pose.orientation.w = 1.0;
    
                // Get the path using the GetPlan service
                nav_msgs::Path path;
                if (getPath(start_pose, goal_pose, path)) {
                    double path_length = calculatePathLength(path);
                    json_file["cost"] = path_length;  // You can calculate the cost if needed
                    // Store the frame_id
                    json_file["frame_id"] = "/map";
    
                    // Store all poses in the JSON file
                    for (auto it = path.poses.rbegin(); it != path.poses.rend(); ++it) {
                        const auto& pose = *it;
                        nlohmann::json pose_json;
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
                    ROS_ERROR("Failed to get path from GetPlan service");
                    continue;
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
            ps.pose.orientation.z = point["qz"].get<double>();
            ps.pose.orientation.w = point["qw"].get<double>();

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
        marker.scale.x = 0.2;  // Point size
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
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

    visualization_msgs::Marker createGoodsMarker(void) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "/global_path/good_points";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.2;  // Point size
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;  
        marker.color.g = 0.0;
        marker.color.b = 1.0;  // Blue color
        marker.color.a = 1.0;
        // marker.text = productName;

        for (const auto &goodName : goodsResult) {
            
            std::string filePath = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/ProductPoseReal/" + goodName + ".json";

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

    visualization_msgs::Marker createClusterMarker(void) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "/global_path/cluster_points";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.2;  // Point size
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;  
        marker.color.g = 1.0;   // Green color
        marker.color.b = 0.0;  
        marker.color.a = 1.0;
        // marker.text = productName;

        for (int32_t idx = goodsIndices[gaResultIndex-1]; idx < goodsIndices[gaResultIndex]; idx++) {
            std::string filePath = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/ProductPoseReal/" 
                                        + goodsResult[idx] + ".json";

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

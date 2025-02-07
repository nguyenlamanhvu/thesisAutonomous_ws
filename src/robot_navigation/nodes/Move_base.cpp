#include "../include/robot_navigation/json.hpp"
#include "../include/robot_navigation/A_star.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <vector>
#include <tuple>
#include <map>
#include "std_msgs/Empty.h"

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
    ros::Subscriber create_astar_result_sub;
    ros::ServiceClient replan_astar_result_client;

    void create_astar_result_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_INFO_STREAM("Create Astar Result Callback");
        createJsonFile();
    }

public:
    MoveBase(void) {
        ros::NodeHandle nh;
        create_astar_result_sub = nh.subscribe("Create_AStar_Result", 10, &MoveBase::create_astar_result_callback, this);

        replan_astar_result_client = nh.serviceClient<robot_navigation::ReplanPath>("replan_astar");
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
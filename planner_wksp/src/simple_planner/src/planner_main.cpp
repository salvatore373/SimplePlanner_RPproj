#include "iostream"
#include "ImageMap.h"
#include "RobotPoses.h"
#include "RvizHelper.h"
#include "UniformCostSearch.h"

#include "ros/ros.h"
#include <ros/master.h>
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"

// DEBUG
#include "Eigen/Dense"
#include <fstream>

ImageMap map;

void saveMatrixToFile(Eigen::MatrixXf map, const std::string& filename = "./output.csv") {
    std::ofstream file(filename);

    if (file.is_open()) {
        // Iterate through the matrix and write each element to the file
        for (int i = 0; i < map.rows(); ++i) {
            for (int j = 0; j < map.cols(); ++j) {
                file << map(i, j);

                // Add a comma unless it's the last element in the row
                if (j < map.cols() - 1) {
                    file << ",";
                }
            }

            // Start a new line after each row
            file << "\n";
        }

        file.close();
        std::cout << "Matrix saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

/**
* The operations to perform once both the initial and goal poses have been received
*/
void onPosesReceived(geometry_msgs::Pose initPose, geometry_msgs::Pose goalPose) {
    // Display the robot on the map
    RvizHelper::displayOnMap(initPose);

    std::cout << map.grid.rows() << "x" << map.grid.cols() << std::endl;

    // Compute the path
    UniformCostSearch search(map);
    // compute distance map (distance of each pixel from closest obstacle) -> col and row deriv -> magnitude map
    Eigen::MatrixXf distanceMap = search.computeDistanceMap(search.map.grid, 50);
    Eigen::MatrixXf magnitudeMap = search.computeMagnitudeDerivative(distanceMap);
    Eigen::MatrixXf rowMap = search.computeRowDerivative(distanceMap);
    Eigen::MatrixXf colMap = search.computeColumnDerivative(distanceMap);

//    std::cout << "Original map" << std::endl;
//    std::cout << map.grid << std::endl;
//
//    std::cout << "Row map" << std::endl;
//    std::cout << rowMap << std::endl;
//
//    std::cout << "Col map" << std::endl;
//    std::cout << colMap << std::endl;
//
//    std::cout << "Magnitude map" << std::endl;
//    std::cout << magnitudeMap << std::endl;
//
//    std::cout << "here" << std::endl;

    saveMatrixToFile(search.map.grid.cast<float>(), "./original_map.csv");
    saveMatrixToFile(distanceMap, "./distance_map.csv");
    saveMatrixToFile(rowMap, "./row_map.csv");
    saveMatrixToFile(colMap, "./col_map.csv");
    saveMatrixToFile(magnitudeMap, "./magnitude_map.csv");

    // TODo: complete
    RvizHelper::displayPath();
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_planner");

    // Display the active topics
//    ros::master::V_TopicInfo topic_infos;
//    ros::master::getTopics(topic_infos);
//    // Log the list of active topics with their details
//    ROS_INFO("Active Topics:");
//    for (const auto& topic : topic_infos) {
//        ROS_INFO("Name: %s, Type: %s", topic.name.c_str(), topic.datatype.c_str());
//    }

    // Retrieve and initialize the map
    map.retrieveMap();

    // Retrieve the initial and goal poses
    RobotPoses poses;
    poses.setOnPoseReceivedCallback(onPosesReceived);
    poses.retrieveInitialPose();
    poses.retrieveGoalPose();

    // DEBUG
    RvizHelper::displayPath();

    // Start the communications
    ros::spin();
    return 0;
}
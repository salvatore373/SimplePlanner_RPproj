#include "iostream"
#include "ImageMap.h"
#include "RobotPoses.h"
#include "RvizHelper.h"
#include "PathFinding.h"

#include "ros/ros.h"
#include <ros/master.h>
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"

#include "Eigen/Dense"
// DEBUG
#include <fstream>

ImageMap map;

void saveMatrixToFile(Eigen::MatrixXf map, const std::string &filename = "./output.csv") {
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
    // Initialize an empty list
    std::list <Eigen::Vector2i> path;

    // Display the robot on the map
    RvizHelper::displayOnMap(initPose, map.mapMetaData.resolution * 3);
    // Remove any previous path on the map
    RvizHelper::displayPath(path, map);

    // Convert the given initial and final position in grid coordinates
    Eigen::Vector2i initialPosition = map.convertWorldToMap((float) initPose.position.x, (float) initPose.position.y);
    Eigen::Vector2i goalPosition = map.convertWorldToMap((float) goalPose.position.x, (float) goalPose.position.y);
    // Compute the path
    PathFinding search(map);
    path = search.performPathFinding(initialPosition, goalPosition);

    // DEBUG saveMatrixToFile(search.voronoiMap, "./capperoVoronoi.csv");

    // DEBUG
//    for (auto e: path) {
//        std::cout << e.x() << "," << e.y() << " -> ";
//    }
//    std::cout << std::endl;

    RvizHelper::displayPath(path, map);
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

    // Start the communications
    ros::spin();
    return 0;
}
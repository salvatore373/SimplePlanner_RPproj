#include "iostream"
#include "ImageMap.h"
#include "RobotPoses.h"
#include "RvizHelper.h"
#include "UniformCostSearch.h"

#include "ros/ros.h"
#include <ros/master.h>
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"

// #include "Eigen/Dense"

ImageMap map;

/**
* The operations to perform once both the initial and goal poses have been received
*/
void onPosesReceived(geometry_msgs::Pose initPose, geometry_msgs::Pose goalPose) {
    // Display the robot on the map
    RvizHelper::displayOnMap(initPose);

    // Compute the path
    UniformCostSearch search(map);
    // compute distance map (distance of each pixel from closest obstacle) -> col and row deriv -> magnitude map
    auto distanceMap = search.computeDistanceMap(search.map.grid);
    auto magnitudeMap = search.computeMagnitudeDerivative(distanceMap);
    auto rowMap = search.computeRowDerivative(distanceMap);
    auto colMap = search.computeColumnDerivative(distanceMap);

    std::cout << "Original map" << std::endl;
    std::cout << map.grid << std::endl;

    std::cout << "Row map" << std::endl;
    std::cout << rowMap << std::endl;

    std::cout << "Col map" << std::endl;
    std::cout << colMap << std::endl;

    std::cout << "Magnitude map" << std::endl;
    std::cout << magnitudeMap << std::endl;

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
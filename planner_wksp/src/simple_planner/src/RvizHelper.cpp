//
// Created by Salvatore  Rago on 12/02/24.
//

#include "RvizHelper.h"

void RvizHelper::displayOnMap(geometry_msgs::Pose initialPose) {
// void RobotDisplayer::displayOnMap() {
    // Create a publisher for the robot marker
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Create a marker message
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;  // Use CUBE for a rectangular robot
    marker.action = visualization_msgs::Marker::ADD;

//    marker.pose.position.x = 0.0;  // Replace with your desired initial pose
//    marker.pose.position.y = 0.0;
//    marker.pose.position.z = 0.0;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 1.0;
    marker.pose = initialPose;

    marker.scale.x = .8;  // Replace with your desired dimensions
    marker.scale.y = .8;
    marker.scale.z = .8;

    marker.color.r = 0.0;  // Replace with your desired color
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Set the marker lifetime (0 indicates persistent)
    marker.lifetime = ros::Duration(0);

    // Wait to be connected to the subscribers
    while (marker_pub.getNumSubscribers() < 1) {
        ros::WallDuration sleep_t(0.1);
        sleep_t.sleep();
    }

    // Publish the marker message
    marker_pub.publish(marker);

    // DEBUG
    std::cout << "robot published" << std::endl;
}

ros::Publisher pathPub;
void RvizHelper::displayPath() {
    ros::NodeHandle nh;
    // Create a publisher for the path
    pathPub = nh.advertise<nav_msgs::Path>("path", 10);


    // Create a path message

    // path.size() = 30
    nav_msgs::Path waypoints;
    waypoints.header.frame_id = "map";
    waypoints.header.stamp = ros::Time::now();
    waypoints.poses.resize(30);

    // DEBUG: 10 steps to fill a square

    for (int i = 0; i < 15; i++)
    {
        // Path finds integers of grid cells, we need to publish waypoints
        // as doubles, and add .05 meters so we aim for middle of the cells
        waypoints.poses[i].header.frame_id = "map";
        waypoints.poses[i].header.stamp = ros::Time::now();
        waypoints.poses[i].pose.position.x = (double)(i) / 10 + .05;
        waypoints.poses[i].pose.position.y = (double)(i) / 10 + .05;
        waypoints.poses[i].pose.position.z = 0;
        waypoints.poses[i].pose.orientation.x = 0;
        waypoints.poses[i].pose.orientation.y = 0;
        waypoints.poses[i].pose.orientation.z = 0;
        waypoints.poses[i].pose.orientation.w = 1;
    }
    // Wait to be connected to the subscribers
    while (pathPub.getNumSubscribers() < 1) {
        ros::WallDuration sleep_t(0.1);
        sleep_t.sleep();
    }

    // TODO: Two calls are needed?
    // pathPub.publish(waypoints);
    pathPub.publish(waypoints);

    // DEBUG
    std::cout << "path published" << std::endl;
}
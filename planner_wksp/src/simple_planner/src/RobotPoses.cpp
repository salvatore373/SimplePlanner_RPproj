//
// Created by Salvatore  Rago on 10/02/24.
//

#include "RobotPoses.h"

using namespace std;

/**
 * Sets the callback function to execute when both the initial and final poses have been received
 * @param poseCallback The callback function (with void return type and 2 arguments of type Pose).
 */
void RobotPoses::setOnPoseReceivedCallback(OnPoseReceivedCallbackType poseCallback) {
    this->poseCallback = poseCallback;
}

void RobotPoses::retrieveInitPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    // Reset the poses if new ones are to be given
    if(initPoseReceived && goalPoseReceived) {
        initPoseReceived = false;
        goalPoseReceived = false;
    }

    // Extract the initial pose from the message
    initialPose = (msg->pose).pose;

    initPoseReceived = true;
    if(goalPoseReceived) {
        poseCallback(initialPose, goalPose);
    }
}

void RobotPoses::retrieveGoalPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // Reset the poses if new ones are to be given
    if(initPoseReceived && goalPoseReceived) {
        initPoseReceived = false;
        goalPoseReceived = false;
    }

    // Extract the initial pose from the message
    goalPose = msg->pose;

    goalPoseReceived = true;
    if(initPoseReceived) {
        poseCallback(initialPose, goalPose);
    }
}

void RobotPoses::retrieveInitialPose() {
    ros::NodeHandle n;
    initPoseRetriever = n.subscribe("initialpose", 1000, &RobotPoses::retrieveInitPose, this);
}

void RobotPoses::retrieveGoalPose() {
    ros::NodeHandle n;
    goalPoseRetriever = n.subscribe("move_base_simple/goal", 1000, &RobotPoses::retrieveGoalPose, this);
}
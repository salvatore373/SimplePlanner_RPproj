//
// Created by Salvatore  Rago on 10/02/24.
//

#ifndef SIMPLE_PLANNER_POSE_H
#define SIMPLE_PLANNER_POSE_H

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/MapMetaData.h"

typedef void (*OnPoseReceivedCallbackType)(geometry_msgs::Pose, geometry_msgs::Pose);

class RobotPoses {
public:
    geometry_msgs::Pose initialPose;
    geometry_msgs::Pose goalPose;

    // void (*poseCallback)(geometry_msgs::Pose, geometry_msgs::Pose);
    OnPoseReceivedCallbackType poseCallback;

    bool initPoseReceived = false;
    bool goalPoseReceived = false;

    void retrieveInitialPose();

    void retrieveGoalPose();

    void setOnPoseReceivedCallback(OnPoseReceivedCallbackType poseCallback);
    // void setOnPoseReceivedCallback(void (*poseCallback)(geometry_msgs::Pose, geometry_msgs::Pose));

private:
    ros::Subscriber initPoseRetriever;
    ros::Subscriber goalPoseRetriever;

    void retrieveInitPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    void retrieveGoalPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
};


#endif //SIMPLE_PLANNER_POSE_H

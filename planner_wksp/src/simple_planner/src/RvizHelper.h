//
// Created by Salvatore  Rago on 12/02/24.
//

#ifndef SIMPLE_PLANNER_ROBOTDISPLAYER_H
#define SIMPLE_PLANNER_ROBOTDISPLAYER_H

#include <iostream>

#include "ImageMap.h"

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

class RvizHelper {
public:
    static void displayOnMap(geometry_msgs::Pose initialPose);

    static void displayPath(std::list <Eigen::Vector2i> gridPath, ImageMap map);
};


#endif //SIMPLE_PLANNER_ROBOTDISPLAYER_H

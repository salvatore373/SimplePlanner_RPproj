//
// Created by Salvatore  Rago on 02/02/24.
//

#ifndef PLANNER_WKSP_IMAGEMAP_H
#define PLANNER_WKSP_IMAGEMAP_H

#include <fstream>
#include <iostream>
#include <arpa/inet.h>

#include "Eigen/Dense"

#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

class ImageMap {
public:
    unsigned int width;
    unsigned int height;
    unsigned int num_rows;
    unsigned int num_cols;

    nav_msgs::MapMetaData mapMetaData;

    Eigen::MatrixXi grid;

    void retrieveMap();

private:
    ros::Subscriber map_metadata_sub;
    ros::Subscriber map_data_sub;

    void retrieveMapMetadata(const nav_msgs::MapMetaData::ConstPtr &msg);

    void fillMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    bool isObstacle(int i, int j);

};

#endif //PLANNER_WKSP_IMAGEMAP_H

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
    int width;
    int height;
    int num_rows;
    int num_cols;

    nav_msgs::MapMetaData mapMetaData;

    Eigen::MatrixXi grid;

    void retrieveMap();

    /*
     * Converts a vector of the world to the coordinate of the corresponding cell in the map.
     */
    inline Eigen::Vector2i convertWorldToMap(float x, float y) {
        int gridY = (int) ((x - mapMetaData.origin.position.x) / mapMetaData.resolution);
        int gridX = num_rows - 1 - ((int) ((y - mapMetaData.origin.position.y) / mapMetaData.resolution));
        return Eigen::Vector2i(gridX, gridY);
    }

    /*
     * Converts the coordinate a cell in the map to the corresponding vector in the world.
     */
    inline Eigen::Vector2f convertMapToWorld(int x, int y) {
        float worldX = (float) (y * mapMetaData.resolution + mapMetaData.origin.position.y);
        float worldY = (float) ((-x - 1 + num_rows) * mapMetaData.resolution + mapMetaData.origin.position.x);

        return Eigen::Vector2f(worldX, worldY);
    }

private:
    ros::Subscriber map_metadata_sub;
    ros::Subscriber map_data_sub;

    void retrieveMapMetadata(const nav_msgs::MapMetaData::ConstPtr &msg);

    void fillMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    bool isObstacle(int i, int j);

};

#endif //PLANNER_WKSP_IMAGEMAP_H

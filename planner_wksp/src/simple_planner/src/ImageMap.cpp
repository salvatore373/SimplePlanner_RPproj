//
// Created by Salvatore  Rago on 02/02/24.
//

#include "ImageMap.h"

using namespace std;

/**
 * Returns whether the cell of the grid at (i, j) contains an obstacle or not.
 * @param i The row index.
 * @param j The column index.
 * @return True if (i,j) contains an obstacle.
 */
bool ImageMap::isObstacle(int i, int j) {
    return grid(i, j) == 100;
}

void ImageMap::retrieveMapMetadata(const nav_msgs::MapMetaData::ConstPtr &msg) {
    mapMetaData = *msg;

    width = msg->width;
    height = msg->height;
    num_rows = height;
    num_cols = width;

    // DEBUG
    cout << "map metadata received" << endl;
}

void ImageMap::fillMap(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    // Check that appropriate width and height have been retrieved
    assert(width > 0 && height > 0);

    // i,j = i * width + j
    grid.resize(num_rows, num_cols);
    for (int i = 0; i < num_rows; i++) {
        for (int j = 0; j < num_cols; j++) {
            // Use this formula to get the map in grid as it is shown in the image
            grid(num_rows - i - 1, j) = msg->data[i * width + j];
        }
    }

    // DEBUG
    cout << "map received" << endl;

    // Display the map
    // cout << grid << endl;
}

void ImageMap::retrieveMap() {
    ros::NodeHandle n;
    map_metadata_sub = n.subscribe("map_metadata", 1000, &ImageMap::retrieveMapMetadata, this);
    map_data_sub = n.subscribe("map", 1000, &ImageMap::fillMap, this);
}
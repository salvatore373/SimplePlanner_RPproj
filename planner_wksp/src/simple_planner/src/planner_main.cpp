#include "iostream"
#include "ImageMap.h"

#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

void retrieveMapMetadata(const nav_msgs::MapMetaData::ConstPtr &msg) {
    cout << "I heard: " << msg->width << endl;
}
void fillMap(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    // cout << "I heard: " << msg->width << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_planner");

    ros::NodeHandle n;
    ros::Subscriber map_metadata_sub = n.subscribe("map_metadata", 1000, retrieveMapMetadata);
    ros::Subscriber map_data_sub = n.subscribe("map", 1000, fillMap);
    ros::spin();

//    const char *path = "/Users/salvatore/Desktop/Università/Development/RobotProgramm"
//                       "ing/ROS/docker_volume/planner_wksp/img.png";
//    ImageMap map = ImageMap(path);

    return 0;
}
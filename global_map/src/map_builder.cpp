//
// Created by kevin on 3/8/18.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <local_map/map_builder.h>
#include <local_map/SaveMap.h>
#include <opt_utils/opt_utils.hpp>

ros::Publisher map_publisher;
local_map::MapBuilder* map_builder_ptr;

void handleLaserScan(sensor_msgs::LaserScan msg)
{
    auto start = hmpl::now();
    map_builder_ptr->grow(msg);
    auto end = hmpl::now();
    std::cout << "update map cost time:" << hmpl::getDurationInSecs(start, end) << "\n";
    map_publisher.publish(map_builder_ptr->getMap());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_map");
    ros::NodeHandle nh("~");

    double map_width;
    double map_height;
    double map_resolution;
    nh.param<double>("map_width", map_width, 500);
    nh.param<double>("map_height", map_height, 500);
    nh.param<double>("map_resolution", map_resolution, 0.2);
    local_map::MapBuilder map_builder(map_width, map_height, map_resolution);
    map_builder_ptr = &map_builder;

    ros::Subscriber scanHandler = nh.subscribe<sensor_msgs::LaserScan>("/ros_lidar", 1, handleLaserScan);
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);

    ros::spin();
}

/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC), 
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: submap_node.cpp
 *   Created on: 12, 14, 2017 
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "internal_grid_map/internal_grid_map.hpp"

int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "submap_node");
    ros::NodeHandle nh("~");
    ros::Publisher publisher =
            nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
    ros::Publisher polygonPublisher =
            nh.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);
    
    hmpl::InternalGridMap igm;
    using namespace grid_map;
    std::string image_dir = ros::package::getPath("internal_grid_map");
    image_dir.append("/data/obstacles.png");
    cv::Mat image = cv::imread(image_dir, CV_8UC1);
    if (image.data == nullptr) {
        std::cout << "image file obstacles.png is not found. Please check your data." << std::endl;
        return 0;
    }
    igm.initializeFromImage(image, 0.1, grid_map::Position::Zero());
    igm.addObstacleLayerFromImage(image, 0.5);
    igm.maps.setFrameId("map");
    
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             igm.maps.getLength().x(), igm.maps.getLength().y(),
             igm.maps.getSize()(0), igm.maps.getSize()(1));
    // Publisher in a loop.
    grid_map::Polygon polygon;
    polygon.setFrameId(igm.maps.getFrameId());
    polygon.addVertex(Position( 0.000,  0.000 ));
    polygon.addVertex(Position( 20.0, 0.000 ));
    polygon.addVertex(Position( 20.0, 20.0 ));
    polygon.addVertex(Position( 10.0, 45.0 ));
    polygon.addVertex(Position( 0.0, 20.0));
    polygon.addVertex(Position( 0.0, 0.0));
    
    geometry_msgs::PolygonStamped p_message;
    grid_map::PolygonRosConverter::toMessage(polygon, p_message);
    
    for (grid_map::PolygonIterator iterator(igm.maps, polygon);
         !iterator.isPastEnd(); ++iterator) {
        igm.maps.at(igm.obs, *iterator) = 255;
    }
    
    
    
    // here test the iterator of gridmap
    
    ros::Rate rate(10);
    while (nh.ok()) {
        // Add data to grid map.
        ros::Time time = ros::Time::now();
        igm.maps.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;
        grid_map::GridMapRosConverter::toOccupancyGrid(
                igm.maps, igm.obs, igm.FREE, igm.OCCUPY, message);
        publisher.publish(message);
        polygonPublisher.publish(p_message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                          message.header.stamp.toSec());
        // Wait for next cycle.
        rate.sleep();
    }
    
    return 0;
}

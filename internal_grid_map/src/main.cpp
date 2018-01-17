#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opt_utils/opt_utils.hpp>
#include "internal_grid_map/internal_grid_map.hpp"

int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "internal_grid_map_node");
    ros::NodeHandle nh("~");
    ros::Publisher publisher =
            nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);

    hmpl::InternalGridMap igm;

    std::string image_dir = ros::package::getPath("internal_grid_map");
    image_dir.append("/data/obstacles.png");
    cv::Mat image = cv::imread(image_dir, CV_8UC1);
    if (image.data == nullptr) {
        std::cout << "image file obstacles.png is not found. Please check your data." << std::endl;
        return 0;
    }
    igm.initializeFromImage(image, 1, grid_map::Position::Zero());
    igm.addObstacleLayerFromImage(image, 0.5);
    igm.maps.setFrameId("map");

    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             igm.maps.getLength().x(), igm.maps.getLength().y(),
             igm.maps.getSize()(0), igm.maps.getSize()(1));
    // Publisher in a loop.

    ros::Rate rate(10);
    while (nh.ok()) {
        // Add data to grid map.
        ros::Time time = ros::Time::now();
        igm.maps.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;
        grid_map::GridMapRosConverter::toOccupancyGrid(
                igm.maps, igm.obs, igm.FREE, igm.OCCUPY, message);
        auto start = hmpl::now();
        igm.updateDistanceLayerCV();
        auto end = hmpl::now();
        std::cout << "fast update time:" << hmpl::getDurationInSecs(start, end) << std::endl;
        start = hmpl::now();
        igm.updateDistanceLayer();
        end = hmpl::now();
        std::cout << "slow update time:" << hmpl::getDurationInSecs(start, end) << std::endl;
        publisher.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                          message.header.stamp.toSec());
        // Wait for next cycle.
        rate.sleep();
    }

    return 0;
}

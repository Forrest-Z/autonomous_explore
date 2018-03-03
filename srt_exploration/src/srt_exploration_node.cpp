//
// Created by kevin on 2/25/18.
//
#include <srt_exploration/SRT.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opt_utils/opt_utils.hpp>


sensor_msgs::LaserScan scan;
bool flag = false;
void handleLaserScan(sensor_msgs::LaserScan msg)
{
    scan = msg;
    flag = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "srt_exploration_node");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
    ros::Subscriber scanHandler = nh.subscribe<sensor_msgs::LaserScan>("/ros_lidar", 1, handleLaserScan);

    // create a node
    Eigen::Vector2d pos; pos(0) = 0; pos(1) = -0;
    hae::SRTNodeParams gmp(pos, 0.2, 0.2, 10.80, 10.80, 0.31, 0.5);
    hae::SRTNode node(nullptr, gmp);

    ros::Rate rate(10);
    while (nh.ok()) {
        ros::spinOnce();
        if(flag == true) {
            auto start = hmpl::now();
            node.import_scan(scan);
            Eigen::Vector2d pos2;
            cv::Point2i new_wp_cell = node.select_next_waypoint();
            auto end = hmpl::now();
            std::cout << "srt_node cost time:" << hmpl::getDurationInSecs(start, end) << "\n";

            node.cell_to_cartesian_global(new_wp_cell.x, new_wp_cell.y, pos2(0), pos2(1));
            std::cout << " aaaa " << pos2 << std::endl;
            node.show_grid_map_color(2, 0.1, "node1");
            node.print_stats();
        }
        rate.sleep();
    }

    return 0;
}
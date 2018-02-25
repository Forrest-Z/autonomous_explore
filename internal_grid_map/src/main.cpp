#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opt_utils/opt_utils.hpp>
#include "internal_grid_map/internal_grid_map.hpp"

hmpl::Pose2D goal_point(14.8, -8.9, 0.28);
grid_map::Index goal_index;
void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    goal_point.position.x = goal->pose.position.x;
    goal_point.position.y = goal->pose.position.y;
    goal_point.orientation = tf::getYaw(goal->pose.orientation);
    if(goal_point.orientation < 0) {
        goal_point.orientation += 2 * M_PI;
    }
}

int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "internal_grid_map_node");
    ros::NodeHandle nh("~");
    ros::Publisher publisher =
            nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);

    std::string image_dir = ros::package::getPath("internal_grid_map");
    image_dir.append("/data/test.jpg");
    cv::Mat image = cv::imread(image_dir, CV_8UC1);
    if (image.data == nullptr) {
        std::cout << "image file obstacles.png is not found. Please check your data." << std::endl;
        return 0;
    }

    hmpl::InternalGridMap igm;
    double map_resolution = 0.2;
    igm.initializeFromImage(image, map_resolution, grid_map::Position::Zero());
    igm.addObstacleLayerFromImage(image, 0.5);
    igm.maps.setFrameId("map");
    igm.vis_.reset(new hmpl::ExplorationTransformVis("exploration_transform"));


    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             igm.maps.getLength().x(), igm.maps.getLength().y(),
             igm.maps.getSize()(0), igm.maps.getSize()(1));
    // Publisher in a loop.

    // callback function for start and goal
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalCb);

    ros::Rate rate(10);
    while (nh.ok()) {
        // Wait for next cycle.
        ros::spinOnce();
        // Add data to grid map.
        ros::Time time = ros::Time::now();
        igm.maps.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;

        auto start = hmpl::now();
        igm.updateDistanceLayerCV();
        auto end = hmpl::now();
        std::cout << "fast update time:" << hmpl::getDurationInSecs(start, end) << std::endl;

        grid_map::Position pose(goal_point.position.x, goal_point.position.y);
        bool flag1 = igm.maps.getIndex(pose, goal_index);
        if(!flag1) {
            std::cout << "goal point is out of map" << '\n';
            continue;
        }
        std::vector<grid_map::Index> tmp;
        tmp.push_back(goal_index);
        start = hmpl::now();
        bool flag = igm.updateExplorationTransform(tmp, 5, 10);
        end = hmpl::now();
        igm.vis_->publishVisOnDemand(igm.maps, igm.explore_transform);
        std::cout << "explore cost time:" << hmpl::getDurationInSecs(start, end) << "\n"
                  << "flag : " << flag  << '\n';

        grid_map::GridMapRosConverter::toOccupancyGrid(
                igm.maps, igm.obs, igm.FREE, igm.OCCUPY, message);
        publisher.publish(message);

        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                          message.header.stamp.toSec());
        // Wait for next cycle.
        rate.sleep();
    }

    return 0;
}

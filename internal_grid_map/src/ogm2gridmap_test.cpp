//
// Created by kevin on 3/18/18.
//

#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/GetMap.h>
#include <grid_map_msgs/GridMap.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opt_utils/opt_utils.hpp>
#include "internal_grid_map/internal_grid_map.hpp"

hmpl::Pose2D goal_point(14.8, -8.9, 0.28);
grid_map::Index goal_index;
nav_msgs::OccupancyGrid binary_ogm_;
ros::ServiceClient mGetBinaryMapClient_;

bool getBinaryMap() {
    if (!mGetBinaryMapClient_.isValid()) {
        return false;
    }

    nav_msgs::GetMap srv;

    if (!mGetBinaryMapClient_.call(srv)) {
        ROS_INFO("Could not get a Binary map.");
        return false;
    }

    binary_ogm_ = srv.response.map;
    unsigned int  mMapWidth = binary_ogm_.info.width;
    unsigned int mMapHeight = binary_ogm_.info.height;
    ROS_INFO("Got new binary map of size %d x %d", mMapWidth, mMapHeight);

    return true;
}

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
    ros::init(argc, argv, "ogm2gridmap_node");
    ros::NodeHandle nh("~");

    mGetBinaryMapClient_ = nh.serviceClient<nav_msgs::GetMap>(std::string("/binary_map"));
    hmpl::InternalGridMap igm;
    igm.vis_.reset(new hmpl::ExplorationTransformVis( "exploration_transform"));

    // callback function for start and goal
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalCb);
    ros::Publisher publisher =
            nh.advertise<nav_msgs::OccupancyGrid>("binary_map", 1, true);
    ros::Publisher gridmap_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    ros::Rate rate(10);
    while (nh.ok()) {
        // Wait for next cycle.
        ros::spinOnce();

        if (!getBinaryMap()) {
            ROS_ERROR("Could not get a Binary map");
            continue;
        }
        publisher.publish(binary_ogm_);
        grid_map::GridMapRosConverter::fromOccupancyGrid(binary_ogm_, igm.obs, igm.maps);
        // value replacement
        grid_map::Matrix& grid_data = igm.maps[igm.obs];
        size_t size_x = igm.maps.getSize()(0);
        size_t size_y = igm.maps.getSize()(1);
        // pre-process
        // 0 : obstacle
        // 255 : free/unknown
        for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
            for (size_t idx_y = 0; idx_y < size_y; ++idx_y) {
                if (0.0 == grid_data(idx_x, idx_y)) {
                    grid_data(idx_x, idx_y) = igm.FREE;
                } else if(100.0 == grid_data(idx_x, idx_y)) {
                    grid_data(idx_x, idx_y) = igm.OCCUPY;
                }
            }
        }

        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                 igm.maps.getLength().x(), igm.maps.getLength().y(),
                 igm.maps.getSize()(0), igm.maps.getSize()(1));
        auto start = hmpl::now();
        igm.updateDistanceLayerCV();
        auto end = hmpl::now();
        std::cout << "fast DT update time [s]:" << hmpl::getDurationInSecs(start, end) << std::endl;

//        // Publish grid map.
//        ros::Time time = ros::Time::now();
//        igm.maps.setTimestamp(time.toNSec());
//        grid_map_msgs::GridMap message;
//        grid_map::GridMapRosConverter::toMessage(igm.maps, message);
//        gridmap_pub.publish(message);
//        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

        grid_map::Position pose(goal_point.position.x, goal_point.position.y);
        bool flag1 = igm.maps.getIndex(pose, goal_index);
        if(!flag1) {
            std::cout << "goal point is out of map" << '\n';
            continue;
        }
        std::vector<grid_map::Index> tmp;
        tmp.push_back(goal_index);
        start = hmpl::now();
        bool flag = igm.updateExplorationTransform(tmp, 5, 10, 1.0);
        end = hmpl::now();
        igm.vis_->publishVisOnDemand(igm.maps, igm.explore_transform);
        std::cout << "PT cost time:" << hmpl::getDurationInSecs(start, end) << "\n"
                  << "flag : " << flag  << '\n';

        // Wait for next cycle.
        rate.sleep();
    }

    return 0;
}

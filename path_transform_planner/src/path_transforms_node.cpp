//
// Created by kevin on 1/11/18.
//
#include "path_transform/path_planning.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <opt_utils/opt_utils.hpp>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <chrono>

hmpl::Pose2D start_point(10, 10, M_PI);
hmpl::Pose2D goal_point(0,0, M_PI);

void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
    start_point.position.x = start->pose.pose.position.x;
    start_point.position.y = start->pose.pose.position.y;
    start_point.orientation = tf::getYaw(start->pose.pose.orientation);
    std::cout << "get initial state." << std::endl;
}

void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    goal_point.position.x = goal->pose.position.x;
    goal_point.position.y = goal->pose.position.y;
    goal_point.orientation = tf::getYaw(goal->pose.orientation);
    std::cout << "get the goal." << std::endl;
}

int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "PT_planner_node");
    ros::NodeHandle nh("~");
    std::string package_dir = ros::package::getPath("path_transform_planner");
    std::string base_dir = "/test/";
    std::string img_dir = "test1";
    cv::Mat img_src = cv::imread(package_dir + base_dir + img_dir + ".jpg", CV_8UC1);

    int rows = img_src.rows;
    int cols = img_src.cols;
    double resolution = 0.2;  // in meter
    hmpl::InternalGridMap in_gm;
    in_gm.initializeFromImage(img_src, resolution, grid_map::Position::Zero());
    in_gm.addObstacleLayerFromImage(img_src, 0.5);
    in_gm.updateDistanceLayerCV();
    in_gm.maps.setFrameId("map");
    in_gm.vis_.reset(new hmpl::ExplorationTransformVis("exploration_transform"));

    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             in_gm.maps.getLength().x(), in_gm.maps.getLength().y(),
             in_gm.maps.getSize()(0), in_gm.maps.getSize()(1));
    // Create publishers and subscribers
    ros::Publisher path_publisher =
            nh.advertise<nav_msgs::Path>("result_path", 1, true);
    ros::Publisher publisher =
            nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);

    // callback function for start and goal
    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, startCb);
    ros::Subscriber end_sub = nh.subscribe("/move_base_simple/goal", 1, goalCb);

    grid_map::Matrix& grid_data = in_gm.maps[in_gm.obs];
    size_t size_x = in_gm.maps.getSize()(0);
    size_t size_y = in_gm.maps.getSize()(1);
    // pre-process
    // 0 : obstacle
    // 255 : free/unknown
    for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
        for (size_t idx_y = 0; idx_y < size_y; ++idx_y) {
            if (in_gm.OCCUPY != grid_data(idx_x, idx_y)) {
                grid_data(idx_x, idx_y) = in_gm.FREE;
            }
        }
    }

    // Publisher in a loop.
    ros::Rate rate(10.0);
    while (nh.ok()) {
        ros::Time time = ros::Time::now();
        // publish the grid_map
        in_gm.maps.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;
        grid_map::GridMapRosConverter::toOccupancyGrid(
                in_gm.maps, in_gm.obs, in_gm.FREE, in_gm.OCCUPY, message);

        grid_map::Index goal_index;
        grid_map::Position pose(goal_point.position.x, goal_point.position.y);
        bool flag = in_gm.maps.getIndex(pose, goal_index);
        if(!flag) {
            std::cout << "goal point is out of map" << '\n';
            continue;
        }
        std::vector<grid_map::Index> tmp;
        tmp.push_back(goal_index);
        auto start = hmpl::now();
        in_gm.updateExplorationTransform(tmp, 5, 10);
        auto end = hmpl::now();
        std::cout << "explore cost time:" << hmpl::getDurationInSecs(start, end) << "\n";
        in_gm.vis_->publishVisOnDemand(in_gm.maps, in_gm.explore_transform);


        start = hmpl::now();
        hmpl::Pose2D revised_start_pose;
        flag = grid_map_path_planning::adjustStartPoseIfOccupied(in_gm.maps, start_point, revised_start_pose,
                                                                 in_gm.obs, in_gm.dis, in_gm.explore_transform);
        if(!flag) {
            std::cout << "start point is out of map" << '\n';
            continue;
        }

        std::vector<geometry_msgs::PoseStamped> result_path;
        grid_map_path_planning::findPathExplorationTransform(in_gm.maps, revised_start_pose,
                                                             result_path,in_gm.obs, in_gm.dis, in_gm.explore_transform);
        end = hmpl::now();
        std::cout << "PT planner cost time:" << hmpl::getDurationInSecs(start, end) << "\n";
        if(!result_path.empty()) {
            nav_msgs::Path path_msg;
            geometry_msgs::PoseStamped pose;
            for (auto &point_itr : result_path) {
                pose = point_itr;
                path_msg.header.frame_id = in_gm.maps.getFrameId();
                path_msg.header.stamp = ros::Time::now();
                pose.header = path_msg.header;
                path_msg.poses.push_back(pose);
            }
            path_publisher.publish(path_msg);
        }
        publisher.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                          message.header.stamp.toSec());

        // Wait for next cycle.
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
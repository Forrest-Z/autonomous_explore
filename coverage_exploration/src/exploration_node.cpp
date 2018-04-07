#include <ros/ros.h>

#include "coverage_exploration/Planner.hpp"
using namespace exploration;

int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "coverage_exploration_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    double min_goal_distance = 25;  //cell distance

    std::string map_frame;
    double vehicle_length, vehicle_width, base2back;
    nh.param<std::string>("recieve_global_map_topic_name", map_frame, "/global_map");
    nh.param<double>("vehicle_length", vehicle_length, 4.9);
    nh.param<double>("vehicle_width", vehicle_width, 1.95);
    nh.param<double>("base2back", base2back, 1.09);

    Config config;
    config.weights = Weights(1, 1, 1, 1, 1);
    config.min_goal_distance = min_goal_distance;
    config.vehicle_length = vehicle_length;
    config.vehicle_width = vehicle_width;
    config.base2back = base2back;
    Planner planner(config);
    // Publisher in a loop.
    ros::Subscriber map_sub = n.subscribe<nav_msgs::OccupancyGrid>(map_frame, 1, &Planner::updateCycle, &planner);

    ros::spin();
    return 0;
}

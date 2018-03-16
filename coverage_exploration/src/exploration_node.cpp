#include <ros/ros.h>

#include "coverage_exploration/Planner.hpp"
using namespace exploration;

int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "coverage_exploration_node");
    ros::NodeHandle nh("~");
    double min_goal_distance = 25;  //cell distance
    double vehicle_width = 2.8;
    double vehicle_length = 4.9;
    double base2back = 1.09;

    // ros::Publisher publisher = nh.advertise<geometry_msgs::PoseStamped>("pose_publisher", 1, true);
    Config config;
    config.weights = Weights(1, 1, 1, 1, 1);
    config.min_goal_distance = min_goal_distance;
    config.vehicle_length = vehicle_length;
    config.vehicle_width = vehicle_width;
    config.base2back = base2back;
    Planner planner(config);
    // Publisher in a loop.
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/global_map", 1,
                                                                    &Planner::updateCycle, &planner);

    ros::spin();
    return 0;
}

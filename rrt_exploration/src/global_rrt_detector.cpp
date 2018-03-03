#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"


#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>


// global variables
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal;
visualization_msgs::Marker points, line;


//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    mapData = *msg;
}

int main(int argc, char **argv) {

    MTRand drand; // double in [0, 1) generator, already init

// generate the same numbers as in the original C test program
    ros::init(argc, argv, "global_rrt_frontier_detector");
    ros::NodeHandle nh;

    // fetching all parameters
    float eta;
    std::string map_topic, base_frame_topic;

    std::string ns;
    ns = ros::this_node::getName();

    ros::param::param<float>(ns + "/eta", eta, 0.5);
    ros::param::param<std::string>(ns + "/map_topic", map_topic, "/local_map/local_map");
//---------------------------------------------------------------
    ros::Subscriber sub = nh.subscribe(map_topic, 100, mapCallBack);

    ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns + "_shapes", 10);

    ros::Rate rate(100);


// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
    while (mapData.header.seq < 1 or mapData.data.size() < 1) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }


//visualizations  points and lines..
    points.header.frame_id = mapData.header.frame_id;
    line.header.frame_id = mapData.header.frame_id;
    points.header.stamp = ros::Time(0);
    line.header.stamp = ros::Time(0);

    points.ns = line.ns = "markers";
    points.id = 0;
    line.id = 1;


    points.type = points.POINTS;
    line.type = line.LINE_LIST;

//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = points.ADD;
    line.action = line.ADD;
    points.pose.orientation.w = 1.0;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.08;
    line.scale.y = 0.08;
    points.scale.x = 0.6;
    points.scale.y = 0.6;

    line.color.r = 9.0 / 255.0;
    line.color.g = 91.0 / 255.0;
    line.color.b = 236.0 / 255.0;
    points.color.r = 9 / 255.0;
    points.color.g = 91 / 255.0;
    points.color.b = 236 / 255.0;
    points.color.a = 1.0;
    line.color.a = 1.0;
    points.lifetime = ros::Duration();
    line.lifetime = ros::Duration();

    float init_map_x, init_map_y, Xstartx, Xstarty;
    init_map_x = init_map_y = 100;
    Xstartx = Xstarty = 0;
    std::vector<std::vector<float> > V;
    std::vector<float> xnew;
    xnew.push_back(0/*trans.x*/);
    xnew.push_back(0/*trans.y*/);
    V.push_back(xnew);

    points.points.clear();
    pub.publish(points);


    std::vector<float> frontiers;
    float xr, yr;
    std::vector<float> x_rand, x_nearest, x_new;

    geometry_msgs::Point p;
// Main loop
    while (ros::ok()) {


// Sample free
        x_rand.clear();
        xr = (drand() * init_map_x) - (init_map_x * 0.5) + Xstartx;
        yr = (drand() * init_map_y) - (init_map_y * 0.5) + Xstarty;


        x_rand.push_back(xr);
        x_rand.push_back(yr);


// Nearest
        x_nearest = Nearest(V, x_rand);

// Steer

        x_new = Steer(x_nearest, x_rand, eta);


// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
        char checking = ObstacleFree(x_nearest, x_new, mapData);

        if (checking == -1) {
            exploration_goal.header.stamp = ros::Time(0);
            exploration_goal.header.frame_id = mapData.header.frame_id;
            exploration_goal.point.x = x_new[0];
            exploration_goal.point.y = x_new[1];
            exploration_goal.point.z = 0.0;
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            points.points.push_back(p);
            pub.publish(points);
            targetspub.publish(exploration_goal);
            points.points.clear();

        } else if (checking == 1) {
            V.push_back(x_new);

            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            line.points.push_back(p);
            p.x = x_nearest[0];
            p.y = x_nearest[1];
            p.z = 0.0;
            line.points.push_back(p);

        }


        pub.publish(line);


        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

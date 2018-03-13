//
// Created by kevin on 2/27/18.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "functions.h"
#include "frontier_search.hpp"

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include "visualization_msgs/MarkerArray.h"
#include <opt_utils/opt_utils.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

typedef std::vector<std::vector<float> > coordVector;
typedef std::vector<coordVector> regionVector;

using namespace frontier_exploration;
// global variables
nav_msgs::OccupancyGrid mapData;
coordVector frontiers, frontiers_show;
regionVector regions;
coordVector centroids;
std::vector<float> current_pos;


int min_frontier_size;
float min_frontier_dist;
int nearest_centroid_index = -1;

// common function
bool included(std::vector<float> coord, regionVector regions) {
    for (int i = 0; i < regions.size(); i++) {
        for (int j = 0; j < regions[i].size(); j++) {
            if (regions[i][j] == coord) {
                return true;
            }
        }
    }
    return false;
}

bool isOnSameFrontier(std::vector<float> coordI, std::vector<float> coordJ, float min_dist) {
    if ((coordI[0] != coordJ[0]) || (coordI[1] != coordJ[1])) {
        float d = sqrt(pow(coordI[0] - coordJ[0], 2) + pow(coordI[1] - coordJ[1], 2));
        if (d <= min_dist) {
            return true;
        } else {
            return false;
        }
    } else {
        // not consider same point
        return false;
    }
}

//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    mapData = *msg;
}


//Subscribers callback functions---------------------------------------
void goalpointCallBack(const geometry_msgs::PointStampedConstPtr & msg) {
    float x,y;
    x = msg->point.x;
    y = msg->point.y;
    std::vector<float> coord;
    coord.push_back(x);
    coord.push_back(y);
    frontiers.push_back(coord);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "filter");
    ros::NodeHandle nh;

    std::string  map_topic, base_frame_topic, goals_topic, base_link_topic;
    int rates;
    std::string ns;
    ns = ros::this_node::getName();
    ros::param::param<int>(ns + "/min_frontier_size", min_frontier_size, 1);
    ros::param::param<float>(ns + "/min_frontier_dist", min_frontier_dist, 2);
    ros::param::param<std::string>(ns + "/map_topic", map_topic, "/global_map");
    ros::param::param<std::string>(ns + "/base_link", base_link_topic, "/base_link");
    ros::param::param<std::string>(ns + "/robot_frame", base_frame_topic, "/odom");
    ros::param::param<std::string>(ns + "/goals_topic", goals_topic, "/detected_points");
    ros::param::param<int>(ns + "/rate", rates, 100);

//---------------------------------------------------------------
    ros::Subscriber map_sub = nh.subscribe(map_topic, 100, mapCallBack);
    ros::Subscriber detected_points_sub = nh.subscribe(goals_topic, 100, goalpointCallBack);

    ros::Publisher frontiers_pub = nh.advertise<visualization_msgs::Marker>(ns + "/frontiers", 10);
    ros::Publisher centroids_pub = nh.advertise<visualization_msgs::MarkerArray>(ns +"/centroids", 10);
    ros::Publisher filteredpoint_pub = nh.advertise<geometry_msgs::PoseArray>("filtered_points", 10);

    ros::Rate rate(rates);

    frontiers.clear();
    regions.clear();
    frontiers_show.clear();


// wait until map is received, when a map is received, mapData.header.seq will not be < 1
    while (mapData.header.seq < 1 or mapData.data.size() < 1 or (0 == frontiers.size())) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // initialize
    frontier_exploration::FrontierSearch searchfrom(mapData);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    int temp = 0;
    while (temp == 0) {
        try {
            temp = 1;
            listener.lookupTransform(base_frame_topic, base_link_topic, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            temp = 0;
            ros::Duration(0.1).sleep();
        }
    }
    double xinit_ =  transform.getOrigin().x();
    double yinit_ =  transform.getOrigin().y();
    current_pos.push_back(0);
    current_pos.push_back(0);

    // Main loop
    while (ros::ok()) {
        ros::spinOnce();
        // get current pos
        tf::StampedTransform transform;
        int temp = 0;
        while (temp == 0) {
            try {
                temp = 1;
                listener.lookupTransform(base_frame_topic, base_link_topic, ros::Time(0), transform);
            } catch (tf::TransformException ex) {
                temp = 0;
                ros::Duration(0.1).sleep();
            }
        }

        // update position
        current_pos.clear();
        current_pos.push_back( transform.getOrigin().x() - xinit_);
        current_pos.push_back( transform.getOrigin().y() - yinit_);


        auto start = hmpl::now();
        regions.clear();
        // merge
        for (int i = 0; i < frontiers.size(); i++) {
            coordVector region = {};
            if (included(frontiers[i], regions) == false) {
                //I proceed only if the current coord has not been already considered
                region.push_back(frontiers[i]);
                for (int j = i + 1; j < frontiers.size(); j++) {

                    for (int k = 0; k < region.size(); k++) {
                        if (isOnSameFrontier(region[k], frontiers[j], min_frontier_dist)) {
                            region.push_back(frontiers[j]);
                            break;
                        }
                    }
                }
                //doesn't consider the failed regions..... (not needed iterations)
                if (region.size() >= min_frontier_size)
                    regions.push_back(region);
            }
        }
//    auto end = hmpl::now();
//    std::cout << "frontier merge  cost time:" << hmpl::getDurationInSecs(start, end) << "\n";

        std::cout << "Frontier Points: " << frontiers.size() << '\n';
        std::cout << "Frontier Regions: " << regions.size() << '\n';

        // compute centroids
        centroids.clear();
        float min_distance = std::numeric_limits<float>::max();

        for (int i = 0; i < regions.size(); i++) {
            float accX = 0;
            float accY = 0;
            for (int j = 0; j < regions[i].size(); j++) {
                accX += regions[i][j][0];
                accY += regions[i][j][1];
            }
            float meanX = accX / regions[i].size();
            float meanY = accY / regions[i].size();
            std::vector<float> centroid;
            centroid.push_back(meanX);
            centroid.push_back(meanY);
            centroids.push_back(centroid);

            float distance = sqrt(pow(meanX-current_pos[0], 2)+pow(meanY-current_pos[1], 2));
            if(distance < min_distance && distance > 5){
                min_distance = distance;
                nearest_centroid_index = centroids.size()-1;
            }
        }
        // update frontiers(no rid of) for frontier publisher
        frontiers_show.clear();
        frontiers_show = centroids;
//        std::cout << "centroids size: " << centroids.size() << '\n';
        if(-1 != nearest_centroid_index) {
            ROS_INFO("Closest centroid (x,y) map: index: %d, %f, %f \t (x,y) current pose: %f, %f  ", nearest_centroid_index, centroids[nearest_centroid_index][0],
                     centroids[nearest_centroid_index][1], current_pos[0], current_pos[1]);
        }

//    start = hmpl::now();
    // delete invaid centroids
        for(auto i = centroids.begin(); i != centroids.end(); ) {
            if(!isFree(mapData, *i, 2.0)|| (informationGain(mapData, *i, 1) < 30)) {
                i = centroids.erase(i);
            } else {
                ++i;
            }
        }
        std::cout << "filtered centroids size: " << centroids.size() << '\n';

        // replace frontier with centroids, lasting iteration
        frontiers.clear();
        frontiers = centroids;

//    end = hmpl::now();
//    std::cout << "frontier fix  cost time:" << hmpl::getDurationInSecs(start, end) << "\n";
//
//    std::cout << "fixed centroids size: " << centroids.size() << '\n';

        auto end = hmpl::now();
        std::cout << "frontier merge cost time:" << hmpl::getDurationInSecs(start, end) << "\n";

        // publishing
        //visualizations  points and lines..
        visualization_msgs::Marker points;
        points.header.frame_id = mapData.header.frame_id;
        points.header.stamp = ros::Time(0);
        points.ns  = "markers2";
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.6;
        points.scale.y = 0.6;
        points.color.r = 255.0 / 255.0;
        points.color.g = 255.0 / 255.0;
        points.color.b = 0.0 / 255.0;
        points.color.a = 0.5;
        points.lifetime = ros::Duration();
        for(int i = 0; i < frontiers_show.size(); i++) {
            geometry_msgs::Point p;
            p.x =  frontiers_show[i][0];
            p.y =  frontiers_show[i][1];
            p.z =  0;
            points.points.push_back(p);
        }
        frontiers_pub.publish(points);

        visualization_msgs::MarkerArray markersMsg;
        geometry_msgs::PoseArray filtered_points;
        filtered_points.header.stamp = ros::Time(0);
        filtered_points.header.frame_id = mapData.header.frame_id;
        for(int i = 0; i < centroids.size(); i++) {
            geometry_msgs::Pose pos;
            visualization_msgs::Marker marker;
            marker.header.frame_id = mapData.header.frame_id;
            marker.header.stamp = ros::Time();
            //marker.ns = "my_namespace";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = pos.position.x = centroids[i][0] ;
            marker.pose.position.y = pos.position.y =centroids[i][1] ;
            marker.pose.position.z = pos.position.z =0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            if(i == nearest_centroid_index) {
                marker.scale.x = 1.5;
                marker.scale.y = 1.5;
                marker.scale.z = 0.2;
                marker.color.a = 1.0;
            } else {
                marker.scale.x = 0.8;
                marker.scale.y = 0.8;
                marker.scale.z = 0.4;
                marker.color.a = 0.5;
            }
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            markersMsg.markers.push_back(marker);
            filtered_points.poses.push_back(pos);

        }
        filteredpoint_pub.publish(filtered_points);
        centroids_pub.publish(markersMsg);
        rate.sleep();
    }
    return 0;

}



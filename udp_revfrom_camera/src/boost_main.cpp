//
// Created by kevin on 4/14/18.
//

#include <string>

#include <ros/ros.h>
#include <iv_explore_msgs/ObjectRecognitionStatus.h>
#include <boostudp.h>

#define BUF_LEN 5 // Larger than maximum UDP packet size

iv_explore_msgs::ObjectRecognitionStatus status;

void receive_data(const char* msg, size_t bytes_transferred) {
    ROS_INFO("Receive MSG Length : %d", bytes_transferred);
    status.object_search_result = msg[0];
}

int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "udp_revfrom_camera_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Publisher recognition_status_pub =
            n.advertise<iv_explore_msgs::ObjectRecognitionStatus>("/object_status", 1);

    std::string ip_local_st = "0.0.0.0";
    nh.param<std::string>("local_ip", ip_local_st, "127.0.0.1");
    int port_local_si = 7778;
    nh.param<int>("local_port", port_local_si, 7778);


    // --- Setup the transmission ---
    BoostUdp transmission(ip_local_st, port_local_si);
//    transmission.connectRemoteEndpoint("192.168.0.111",9905);
    char ip_local_scp[1024];
    transmission.setReadCallback(boost::bind(&receive_data, _1, _2));
    // Publisher in a loop.
    ros::Rate rate(10.0);
    while (nh.ok()) {
        transmission.receive();
        recognition_status_pub.publish(status);
        // Wait for next cycle.
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

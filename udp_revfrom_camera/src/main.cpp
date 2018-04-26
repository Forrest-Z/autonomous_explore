#include <string>

#include <ros/ros.h>
#include <iv_explore_msgs/ObjectRecognitionStatus.h>
#include <data_transmission.h>

#define BUF_LEN 5 // Larger than maximum UDP packet size


int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "udp_revfrom_camera_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    std::string se_object_send_topic_name;
    nh.param<std::string>("send_object_send_topic_name", se_object_send_topic_name, "/object_status");
    ros::Publisher recognition_status_pub =
            n.advertise<iv_explore_msgs::ObjectRecognitionStatus>(se_object_send_topic_name, 1);

    std::string ip_local_st = "0.0.0.0";
    nh.param<std::string>("local_ip", ip_local_st, "127.0.0.1");
    int port_local_si = 7778;
    nh.param<int>("local_port", port_local_si, 7778);


    // --- Setup the transmission ---
    data_transmission transmission;
    char ip_local_scp[1024];
    strcpy(ip_local_scp, ip_local_st.c_str());
    int fd = transmission.init_transmission(ip_local_scp, port_local_si);
    if(fd < 0) {
        ROS_ERROR("cannot create socket");
        return 0;
    }else {
//       if(false == transmission.setNonBlocking(fd) ) {
//           ROS_ERROR("cannot enable nonblocking!");
//           return 0;
//       }
    }
    iv_explore_msgs::ObjectRecognitionStatus status;
    // Publisher in a loop.
    ros::Rate rate(10.0);
    while (nh.ok()) {
        char message[BUF_LEN];
        // Block until receive message from a client
        transmission.listen(message, sizeof(uint8_t));
        ROS_INFO_THROTTLE(3, "Receive object recognition flag : %d", message[0]);
        status.object_search_result = message[0];
        recognition_status_pub.publish(status);
        // Wait for next cycle.
	    ros::spinOnce();
        rate.sleep();
    }

    transmission.close_transmission();
    return 0;
}

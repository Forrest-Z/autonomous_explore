#ifndef POSE_HANDLER_H
#define POSE_HANDLER_H

#include <tf/transform_listener.h>

namespace tfhandler{

class PoseHandler
{
public:
  PoseHandler() {

  }

  tf::StampedTransform lookupPose(std::string parent="/map", std::string child="/base_link") {
      tf::StampedTransform transform;
      try{
          listener.lookupTransform(parent, child, ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
      }

      return transform;
  }

private:
    tf::TransformListener listener;

};

}
#endif // POSE_HANDLER_H

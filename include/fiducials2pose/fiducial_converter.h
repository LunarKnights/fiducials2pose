#ifndef FIDUCIALS2POSE_FIDUCIAL_CONVERTER_H
#define FIDUCIALS2POSE_FIDUCIAL_CONVERTER_H

#include "ros/ros.h"

#include <vector>
#include <string>

#include <nodelet/nodelet.h>

#include <fiducial_msgs/FiducialTransformArray.h>

#include <tf2_ros/transform_broadcaster.h>

namespace fiducials2pose {

class FiducialConverter: public nodelet::Nodelet
{
public:
  FiducialConverter();
  void onInit() override;
protected:
  void fiducialCb(const fiducial_msgs::FiducialTransformArray &msg);
  std::string cameraFrame;
  std::vector<std::string> fiducialFrames;
  ros::Subscriber poseSub;
  tf2_ros::TransformBroadcaster broadcaster;
};


}

#endif

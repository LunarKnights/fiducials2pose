#include "ros/ros.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <fiducial_msgs/FiducialTransformArray.h>

#include <geometry_msgs/TransformStamped.h>

#include "fiducials2pose/fiducial_converter.h"

PLUGINLIB_EXPORT_CLASS(fiducials2pose::FiducialConverter, nodelet::Nodelet);

namespace fiducials2pose {

FiducialConverter::FiducialConverter()
{
  // do nothing
}

void FiducialConverter::onInit()
{
  auto nhPrivate = getPrivateNodeHandle();
  nhPrivate.param("camera_frame_id", cameraFrame, std::string("camera"));
  std::string fiducialTopicName;
  nhPrivate.param("fiducial_topic", fiducialTopicName, std::string("fiducials"));
  std::string tfPrefix;

  if (!nhPrivate.getParam("fiducial_frame_ids", fiducialFrames))
  {
    NODELET_FATAL("fiducial_frame_ids must be provided");
    return;
  }

  poseSub = nhPrivate.subscribe(fiducialTopicName, 1, &FiducialConverter::fiducialCb, this);
}

void FiducialConverter::fiducialCb(const fiducial_msgs::FiducialTransformArray &msg)
{
  for (const auto &ft: msg.transforms)
  {
    if (ft.fiducial_id >= fiducialFrames.size())
    {
      ROS_WARN("found fiducial without matching name %d", ft.fiducial_id);
      continue;
    }
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = msg.header.stamp;
    ts.header.frame_id = cameraFrame;
    ts.child_frame_id = fiducialFrames[ft.fiducial_id];
    ts.transform = ft.transform;

    broadcaster.sendTransform(ts);
  }
}

}


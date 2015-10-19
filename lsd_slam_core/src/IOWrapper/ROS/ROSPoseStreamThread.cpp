#include "ROSPoseStreamThread.h"

#include <boost/thread.hpp>
#include <geometry_msgs/Pose.h>

#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"
#include "IOWrapper/InputPoseStream.h"

namespace lsd_slam {

ROSPoseStreamThread::ROSPoseStreamThread() {
  odom_channel = nh_.resolveName("odom_gt");
  odom_sub     = nh_.subscribe(odom_channel, 1, &ROSPoseStreamThread::odomCb, this);

  poseBuffer = new NotifyBuffer<TimestampedPose>(100);

}

ROSPoseStreamThread::~ROSPoseStreamThread() {
  delete poseBuffer;
}

void ROSPoseStreamThread::run() {
  boost::thread thread(boost::ref(*this));
}

void ROSPoseStreamThread::operator()() {
  ros::spin();
  exit(0);
}

void ROSPoseStreamThread::odomCb(const nav_msgs::OdometryConstPtr odom) {

  TimestampedPose bufferItem;
  bufferItem.timestamp = Timestamp(odom->header.stamp.toSec());

  geometry_msgs::Pose pose;
  pose.position.x = odom->pose.pose.position.x;
  pose.position.y = odom->pose.pose.position.y;
  pose.position.z = odom->pose.pose.position.z;

  pose.orientation.x = odom->pose.pose.orientation.x;
  pose.orientation.y = odom->pose.pose.orientation.y;
  pose.orientation.z = odom->pose.pose.orientation.z;
  pose.orientation.w = odom->pose.pose.orientation.w;

  bufferItem.data = pose;

  poseBuffer->pushBack(bufferItem);

}

}

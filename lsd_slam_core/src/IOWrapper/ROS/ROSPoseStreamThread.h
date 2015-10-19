#ifndef ROSPOSESTREAMTHREAD_H
#define ROSPOSESTREAMTHREAD_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "IOWrapper/InputPoseStream.h"

namespace lsd_slam {

class ROSPoseStreamThread : public InputPoseStream {
 public:
  ROSPoseStreamThread();
  ~ROSPoseStreamThread();

  void run();

  void operator()();

  void odomCb(const nav_msgs::OdometryConstPtr odom);

 private:
  ros::NodeHandle nh_;

  std::string     odom_channel;
  ros::Subscriber odom_sub;

};

}

#endif

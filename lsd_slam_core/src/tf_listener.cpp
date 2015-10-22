#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Image.h>

#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "tf_node");

  ros::NodeHandle nh("~");

  tf::TransformListener listener;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher debug_pub = nh.advertise<geometry_msgs::Vector3>("debug", 1);

  ros::Rate rate(100.0);

  std::string frame_id         = "openni_camera"; // "world";
  std::string child_frame_id   = "openni_rgb_optical_frame";

  // std::string frame_id         = "world"; // "world";
  // std::string child_frame_id   = "kinect";

  while (ros::ok()) {
    tf::StampedTransform transform;

    try {
      listener.lookupTransform(frame_id, child_frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      continue;
    }

    static ros::Time pTime = transform.stamp_;
    ros::Duration diff = transform.stamp_ - pTime;
    if (diff.toSec() * 1000 < 1) {
      ROS_WARN("same tf %f %f", pTime.toSec(), transform.stamp_.toSec());
      continue;
    }
    pTime = transform.stamp_;

    nav_msgs::Odometry odom;
    odom.header.stamp            = transform.stamp_;
    odom.header.frame_id         = frame_id;
    odom.child_frame_id          = child_frame_id;

    // tf::poseStampedTFToMsg(transform, odom.pose.pose);
    odom.pose.pose.position.x    = transform.getOrigin().getX();
    odom.pose.pose.position.y    = transform.getOrigin().getY();
    odom.pose.pose.position.z    = transform.getOrigin().getZ();

    odom.pose.pose.orientation.w = transform.getRotation().getW();
    odom.pose.pose.orientation.x = transform.getRotation().getX();
    odom.pose.pose.orientation.y = transform.getRotation().getY();
    odom.pose.pose.orientation.z = transform.getRotation().getZ();

    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

    geometry_msgs::Vector3 debug;
    debug.x = roll * 180 / M_PI;
    debug.y = pitch * 180 / M_PI;
    debug.z = yaw * 180 / M_PI;
    debug_pub.publish(debug);

    odom_pub.publish(odom);

    rate.sleep();
  }

  return 0;
}

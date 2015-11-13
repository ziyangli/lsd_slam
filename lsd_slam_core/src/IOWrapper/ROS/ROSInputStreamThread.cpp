#include <boost/thread.hpp>

#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/image_encodings.h>

#include "ROSInputStreamThread.h"

namespace lsd_slam {

using namespace cv;

ROSInputStreamThread::ROSInputStreamThread() {
  // subscribe
  vid_channel  = nh_.resolveName("image");
  vid_sub      = nh_.subscribe(vid_channel, 1, &ROSInputStreamThread::imgCb, this);

  odom_channel = nh_.resolveName("odom");
  odom_sub     = nh_.subscribe(odom_channel, 1, &ROSInputStreamThread::odomCb, this);

  // wait for cam calib
  width_ = height_ = 0;

  // imagebuffer
  imageBuffer = new NotifyBuffer<TimestampedMat>(50);
  poseBuffer  = new NotifyBuffer<TimestampedTFMsg>(10);

  undistorter = NULL;
  lastSEQ     = 0;

  haveCalib   = false;
}

ROSInputStreamThread::~ROSInputStreamThread() {
  delete imageBuffer;
  delete poseBuffer;
}

void ROSInputStreamThread::run() {
  boost::thread thread(boost::ref(*this));
}

void ROSInputStreamThread::operator()() {
  ros::spin();
  exit(0);
}

void ROSInputStreamThread::odomCb(const nav_msgs::OdometryConstPtr& odom) {
  geometry_msgs::Transform pose;
  pose.translation.x = odom->pose.pose.position.x;
  pose.translation.y = odom->pose.pose.position.y;
  pose.translation.z = odom->pose.pose.position.z;
  pose.rotation.x    = odom->pose.pose.orientation.x;
  pose.rotation.y    = odom->pose.pose.orientation.y;
  pose.rotation.z    = odom->pose.pose.orientation.z;
  pose.rotation.w    = odom->pose.pose.orientation.w;

  TimestampedTFMsg bufferItem;
  bufferItem.timestamp = Timestamp(odom->header.stamp.toSec());
  bufferItem.data      = pose;

  poseBuffer->pushBack(bufferItem);
}

void ROSInputStreamThread::imgCb(const sensor_msgs::ImageConstPtr& img) {
  if (!haveCalib) return;

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

  assert(img->header.seq > lastSEQ);
  lastSEQ = img->header.seq;

  TimestampedMat bufferItem;
  bufferItem.timestamp = Timestamp(img->header.stamp.toSec());

  if (undistorter != NULL) {
    assert(undistorter->isValid());
    undistorter->undistort(cv_ptr->image(cv::Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows/3)), bufferItem.data);
  }
  else {
    bufferItem.data = cv_ptr->image(cv::Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows/3));
  }

  imageBuffer->pushBack(bufferItem);
}

void ROSInputStreamThread::infoCb(const sensor_msgs::CameraInfoConstPtr& info) {

  if (!haveCalib) {
    fx_ = info->P[0];
    fy_ = info->P[5];
    cx_ = info->P[2];
    cy_ = info->P[6];

    if (fx_ == 0 || fy_==0) {
      printf("camera calib from P seems wrong, trying calib from K\n");
      fx_ = info->K[0];
      fy_ = info->K[4];
      cx_ = info->K[2];
      cy_ = info->K[5];
    }

    width_  = info->width;
    height_ = info->height;

    printf("Received ROS Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n", fx_, fy_, cx_, cy_, width_, height_);
  }
}

void ROSInputStreamThread::setCalibration(std::string file) {
  if (file == "") {
    ros::Subscriber info_sub = nh_.subscribe(nh_.resolveName("camera_info"), 1, &ROSInputStreamThread::infoCb, this);

    printf("WAITING for ROS camera calibration!\n");
    while (width_ == 0) {
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.03));
    }
    printf("RECEIVED ROS camera calibration!\n");

    info_sub.shutdown();
  }
  else {
    undistorter = Undistorter::getUndistorterForFile(file.c_str());

    if (undistorter == 0) {
      printf("Failed to read camera calibration from file... wrong syntax?\n");
      exit(0);
    }

    fx_     = undistorter->getK().at<double>(0, 0);
    fy_     = undistorter->getK().at<double>(1, 1);
    cx_     = undistorter->getK().at<double>(2, 0);
    cy_     = undistorter->getK().at<double>(2, 1);

    width_  = undistorter->getOutputWidth();
    height_ = undistorter->getOutputHeight();
  }

  haveCalib = true;
}

}

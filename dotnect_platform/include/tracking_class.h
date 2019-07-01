// file: tracking_class.h, style: README.md
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// Info inside tracking_class.cpp
//
// preproc
#ifndef TRACKING_CLASS_H
#define TRACKING_CLASS_H
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
// boost
#include <boost/atomic.hpp>
#include <boost/thread/mutex.hpp>
// ros
#include <image_transport/image_transport.h>
#include <ros/ros.h>
// opencv
#include <opencv2/core/mat.hpp>
// local
#include "data_container.h"

class TrackingClass
{
public:
  TrackingClass(DataContainer* container);
  ~TrackingClass();
  void subscribeImgSrc(std::string topic_name);
  bool setVerbose(bool& verbose);
  void callbImg(const sensor_msgs::ImageConstPtr& img_ptr);

private:
  int u_;  // col
  int v_;  // row
  unsigned short depth_;
  double fx_;  // intrinsics of camera_matrix
  double fy_;
  double cx_;
  double cy_;
  double x_;  // 3D point
  double y_;
  double z_;
  std::vector<double> point_f1_;
  boost::atomic<bool> verbose_;
  boost::mutex mtx_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_img_transport_;
  image_transport::Subscriber sub_img_transport_;
  cv::Mat mat_CV8UC1_;
  cv::Mat mat_CV8UC1_flip_;
  DataContainer* container_;
};
#endif
// EOF

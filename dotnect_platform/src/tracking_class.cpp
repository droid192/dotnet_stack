// file: tracking_class.cpp, style: README.md
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// Get input matrix, filter it_ and do minimum entry search (MES)
// Then project found MES-pixel via intrinsics to 3D point and publish
// Also paint found MES-pixel into image and publish it
//
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// boost
#include <boost/thread/mutex.hpp>
// opencv
#include <opencv2/core/mat.hpp>
// local
#include "data_container.h"
#include "tracking_class.h"
#include "custom_defines.h"

// Subscribe to video topic, advertise 3D point/marker, set kinect intrinsics
TrackingClass::TrackingClass(DataContainer *container) : container_(container), it_(nh_)
{
  verbose_ = false;
  point_f1_.resize(6);
  point_f1_[1] = 15;
  // kinect1 intrinsics, adjust fx_,fy_,cx_,cy_ to your camera
  fx_ = 5.9421480358642339e+02;
  fy_ = 5.9104092248505947e+02;
  cx_ = 3.3930546187516956e+02;
  cy_ = 2.4273843891390746e+02;
  // publish image with point position
  pub_img_transport_ = it_.advertise("/dotnect_platform/paint_img", 1);
}

TrackingClass::~TrackingClass()
{
}

// change topic
void TrackingClass::subscribeImgSrc(std::string topic_name)
{
  sub_img_transport_ = it_.subscribe(topic_name, 1, &TrackingClass::callbImg, this);
}

// flag to printf pixel position and depth_ value
bool TrackingClass::setVerbose(bool &verbose)
{
  this->verbose_ = verbose;
}

// image callback: convert, prepare then MES
void TrackingClass::callbImg(const sensor_msgs::ImageConstPtr &img_ptr)
{
  boost::mutex::scoped_lock lock(mtx_);
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // mutable matrix is needed, so copy
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception &e)
  {
    printf(ACRED NNAME ": %s" ACRESETN, e.what());
    return;
  }

  // get pointer directly to cv::Mat member
  cv::Mat *mat_ptr = &(cv_ptr->image);

  // push 0-entries into background having 4000mm along z_-axis (camera depth maximum)
  // layout for img_ptr, cv_ptr, mat_ptr and mat_CV8UC1_;
  // mat_CV8UC1_flip_ inverts the horizontal direction
  //              ________ j,u
  //              |
  //              |  x__.
  //          i,v |     |
  //              |     y
  ushort *p_row;
  // speed: first row, then column
  for (int i = 0; i < mat_ptr->rows; ++i)
  {
    p_row = mat_ptr->ptr<ushort>(i);
    for (int j = 0; j < mat_ptr->cols; ++j)
    {
      if (p_row[j] == 0)
        p_row[j] = 4000;
    }
  }
  // precaution blur
  blur(*mat_ptr, *mat_ptr, cv::Size(3, 3));

  // find minimum = closest point, MES
  depth_ = mat_ptr->at<ushort>(0, 0);
  for (int i = 0; i < mat_ptr->rows; ++i)
  {
    p_row = mat_ptr->ptr<ushort>(i);
    for (int j = 0; j < mat_ptr->cols; ++j)
    {
      if (p_row[j] < depth_)
      {
        u_ = j;
        v_ = i;
        depth_ = p_row[j];
      }
    }
  }

  // pinhole model to project into camera-kosy [mm] (plumb_bob)
  // http://wiki.ros.org/image_pipeline/CameraInfo
  x_ = (u_ - cx_) / fx_ * depth_;
  y_ = (v_ - cy_) / fy_ * depth_;
  z_ = double(depth_);

  // send to container
  point_f1_[0] = ros::Time::now().toNSec();
  point_f1_[2] = cv_ptr->header.seq;
  point_f1_[3] = x_;
  point_f1_[4] = y_;
  point_f1_[5] = z_;
  container_->setVec(point_f1_);

  // option to output pixel position
  if (verbose_)
  {
    printf(NNAME ": x:%f y:%f z:%f; px u:%d v:%d d:%d\n", x_, y_, z_, u_, v_, depth_);
  }

  // convert to greyscale cv::Mat with 8-Bit depth_, (max - min)
  (*mat_ptr).convertTo(mat_CV8UC1_, CV_8UC1, 255. / (2500 - 0), 0);
  cv::flip(mat_CV8UC1_, mat_CV8UC1_flip_, 1);

  // paint black and white point into image
  cv::circle(mat_CV8UC1_flip_, cv::Point(mat_CV8UC1_flip_.cols - 1 - u_, v_), 9, cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
  cv::circle(mat_CV8UC1_flip_, cv::Point(mat_CV8UC1_flip_.cols - 1 - u_, v_), 5, cv::Scalar(255, 255, 255), 3, 8, 0);

  // output modified video stream
  pub_img_transport_.publish(
      cv_bridge::CvImage((*cv_ptr).header, sensor_msgs::image_encodings::TYPE_8UC1, mat_CV8UC1_flip_).toImageMsg());
}
// EOF

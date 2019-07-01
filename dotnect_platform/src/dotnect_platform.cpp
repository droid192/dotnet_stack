// file: dotnect_platform.cpp, style: README.me
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// -subscribes to dotnect_dgram_socket to get orientation and to kinect_tracking to get translation (3D point)
// -averages the 3D kinect point via moving averages: SMA, LMA and EMA (moving_average.h)
// -the information is published again on ros topics
// -implements the dotnect_markers menu actions
//
// NAME     DESCRIPTION
// dotnect_base_link  map coordinate system (frame) to latch dotnect_base_link
// frame_0   robot base
// frame_1   camera body
// frame_2   android refrence used by the rotation-vector sensor
// frame_3   smartphone body

// qr0to2    quaternion that rotates frame_0 to frame_2 (each cosy unit vector of frame_2) coordinate-rotation or
//           quaternion that describes frame_2 relative to frame_0 from a coordinate-transformation point of view;
//
// q0to2     quaternion that describes frame_0 relative to frame_2 from a coordinate-transformation point of view;
//           transformation of a vector v from f0 to f2 would be 2^v = 2^0_R_H(q0to2) * 0^v (Syntax: Craig -
//           Introdcution to Robotics)
//           q0to2 and qr0to2 are conjugates to each other and their rotation matrices derived by R_H(q) are the
//           transpose of one another
//
//           The conversion of q to euler angles (XYZ) is taken from https://github.com/ItsmeJulian/quat2eul
//           The conversion of q to rotation matrix would use the euler-hamilton function R_H(qr0to2) to get 3x3 rotation
//            matrix can found in
//             -matlab quat2rotm
//             -wolfram alpha e.g. https://www.wolframalpha.com/input/?i=quaternion:+0.7071%2B0i%2B0j%2B0.7071k
//             -wikipedia
//             https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
//
#include <stdio.h>
#include <cstring>
// boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
// ros
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// local
#include "custom_defines.h"
#include "data_container.h"
#include "dotnect_custom_msgs/DotnectUi.h"
#include "dotnect_custom_msgs/Float64Array.h"
#include "dotnect_custom_msgs/Int32Array.h"
#include "moving_average.h"
#include "tracking_class.h"
#include "val3_fstream_socket.h"

using namespace std;

// output user args
void printInfo(char **argv)
{
  printf(ACCYAN "$ dotnect_platform <1:int> <2:int> <3:int> <4:string> <5-7:dbl dbl dbl> "
                "<8-10:dbl dbl dbl>" ACRESETN);
  printf("     <1:xma> decide for moving average type (XMA)\n");
  printf("             0: simple moving average SMA\n");
  printf("             1: linear moving average LMA\n");
  printf("             2: exponential moving average EMA (alpha adjust via @FLAG_B)\n");
  printf("     <2:pos/vel> 0: position is averaged\n");
  printf("                 1: position delta = position now - position before\n");
  printf("                 2: velocity\n");
  printf("                 3: velocity delta\n");
  printf("     <3:nPoints> how many past measurements to consider (only SMA/LMA, if XMA set to 1)\n");
  printf("                 (low: fast respond, high: gradual respond)\n");
  printf("                 estimate: nPoints = freq *  time_intervall (e.g. 30 nPoints = 30Hz *1s)\n\n");
  printf("     <4:show_urdf_>  0: disable robot-model/IK-solver/jump-range-validation, show hovering smartphone "
         "instead\n");
  printf("                   1: enbale ...\n");
  printf("     <5-7:offset> initial end-effector position relative to robots base frame_0\n");
  printf("     <8-10:scale> scale end-effector reaction relative to kinect point delta (default: 1 1 1)\n");
}

// subscription name pool
string getSensorTopic(int type) {
  switch (type) {
    case 1: return "/dotnect_dgram_socket/acc";
    case 2: return "/dotnect_dgram_socket/mag";
    case 3: return "/dotnect_dgram_socket/orient";
    case 4: return "/dotnect_dgram_socket/gyro";
    case 5: return "/dotnect_dgram_socket/light";
    case 6: return "/dotnect_dgram_socket/pres";
    case 7: return "/dotnect_dgram_socket/temp";
    case 8: return "/dotnect_dgram_socket/prox";
    case 9: return "/dotnect_dgram_socket/grav";
    case 10: return "/dotnect_dgram_socket/linacc";
    case 11: return "/dotnect_dgram_socket/rot";
    case 12: return "/dotnect_dgram_socket/humid";
    case 13: return "/dotnect_dgram_socket/ambtmp";
    case 14: return "/dotnect_dgram_socket/rot_euler";
    default:
      printf(ACYELLOW NNAME": topic name not stored" ACRESETN);
      return "";
  }
}

// static global object
DataContainer *getContainerObj()
{
  static DataContainer x;
  return &x;
}

// static global object
Val3FstreamSocket *getGenerateVal3()
{
  static Val3FstreamSocket x;
  return &x;
}

// callbacks from dotnect_markers
void callbInt32A(const dotnect_custom_msgs::Int32Array::ConstPtr &msg_)
{
  printf(ACYELLOW NNAME": marker handle state changed" ACRESETN);
}

// callbacks from subscriptions
void callbMAFloat64(const std_msgs::Float64MultiArray::ConstPtr &msg_)
{
  if ((msg_->data)[2] == 2)
  {
    if ((msg_->data)[1] == 11)
    {
      // get real part of rotation-vector sensor to have complete quaternion vector
      static vector<double> tmp;
      double qw =
          sqrt(1 - (msg_->data)[3] * (msg_->data)[3] - (msg_->data)[4] * (msg_->data)[4] - (msg_->data)[5] * (msg_->data)[5]);
      tmp = msg_->data;
      tmp.push_back(qw);
      getContainerObj()->setVec(tmp);
    }
    else
    {
      // data sets into v_dbls_
      getContainerObj()->setVec(msg_->data);
    }
  }
  else if ((msg_->data)[2] == 1)
  {
    // data sets into v_ints_
    getContainerObj()->setInt((msg_->data)[1], (msg_->data)[3]);
  }
  else if ((msg_->data)[2] == 0)
  {
    // data is processed immideately
    getGenerateVal3()->manageSocket((msg_->data)[3]);
  }
}

// let ros flush data to our subscriptions
void ros_spin()
{
  ros::spin();
}

// publish joint states for possible urdf models
class JointsPublisher
{
public:
  JointsPublisher(Val3FstreamSocket *val3_fstream_socket_) : val3_fstream_socket_(val3_fstream_socket_)
  {
    pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states_test", 5);
    joint_state_msg_.name.resize(6);
    joint_state_msg_.name[0] = "a1";
    joint_state_msg_.name[1] = "a2";
    joint_state_msg_.name[2] = "a3";
    joint_state_msg_.name[3] = "a4";
    joint_state_msg_.name[4] = "a5";
    joint_state_msg_.name[5] = "a6";
    joint_state_msg_.position.resize(6);
  }
  ~JointsPublisher()
  {
  }
  void publishJoints(vector<double> &radAng)
  {
    if (radAng.size() != 6)
    {
      printf(ACRED NNAME": joints size %lu" ACRESET, radAng.size());
      return;
    }
    write_lock_(mtx_);
    joint_state_msg_.position[0] = radAng[0];
    joint_state_msg_.position[1] = radAng[1];
    joint_state_msg_.position[2] = radAng[2];
    joint_state_msg_.position[3] = radAng[3];
    joint_state_msg_.position[4] = radAng[4];
    joint_state_msg_.position[5] = radAng[5];
    pub_.publish(joint_state_msg_);
  }

private:
  sensor_msgs::JointState joint_state_msg_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  Val3FstreamSocket *val3_fstream_socket_;
  boost::shared_mutex mtx_;
  typedef boost::unique_lock<boost::shared_mutex> write_lock_;
};

// static global object
JointsPublisher *getJointStatePublisher()
{
  static JointsPublisher x(getGenerateVal3());
  return &x;
}

// Class used for message generation, advertising and publishing
// Each topic has one object, so not multi-threadsafe
class MultiArrayPublisher
{
public:
  MultiArrayPublisher(DataContainer *container, string topic_name, int k) : container_(container), k_(k)
  {
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_name, 10);
    printf(NNAME ": advertised %s %s \n", k == 0 ? "timed" : "", topic_name.c_str());
    msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_.layout.dim[0].stride = 1;
    msg_.layout.dim[0].label = topic_name;
    msg_timed_ = msg_;
  }
  ~MultiArrayPublisher()
  {
  }
  // convert quaternion to euler angles of xyz sequence
  // extract from github.com/ItsmeJulian/quattoEul
  void quattoEul(double &qx, double &qy, double &qz, double &qw, bool toEulDeg)
  {
    double psi = atan2(2 * (qx * qw - qy * qz), (qw * qw - qx * qx - qy * qy + qz * qz));
    double theta = asin(2 * (qx * qz + qy * qw));
    double phi = atan2(2 * (qz * qw - qx * qy), (qw * qw + qx * qx - qy * qy - qz * qz));
    if (toEulDeg)
    {
      psi *= 180 / M_PI;
      theta *= 180 / M_PI;
      phi *= 180 / M_PI;
    }
    qx = psi;
    qy = theta;
    qz = phi;
    qw = 0.;
  }
  // publish position and rotation (quaternion or euler-angles)
  void publish(double x, double y, double z, double qx, double qy, double qz, double qw, bool toEul, bool toEulDeg)
  {
    // convert to xyz euler angles
    if (toEul)
    {
      quattoEul(qx, qy, qz, qw, toEulDeg);
      msg_.layout.dim[0].size = 6;
    }
    else
    {
      msg_.layout.dim[0].size = 7;
    }
    msg_.data.push_back(x);
    msg_.data.push_back(y);
    msg_.data.push_back(z);
    msg_.data.push_back(qx);
    msg_.data.push_back(qy);
    msg_.data.push_back(qz);
    if (toEul == false)
    {
      msg_.data.push_back(qw);
    }
    pub_.publish(msg_);
  }
  // ros timer triggered
  void publishT(const ros::TimerEvent &)
  {
    if (container_->checkVecFlag(4, k_) == 1)
    {
      container_->getVec(4, k_, v_);
    }
    msg_timed_.layout.dim[0].size = v_.size();
    msg_timed_.data = v_;
    pub_.publish(msg_timed_);
  }
  void publish_joint_state()
  {
    // IK-solver not impl.
  }
private:
  int k_;
  vector<double> v_;
  std_msgs::Float64MultiArray msg_;
  std_msgs::Float64MultiArray msg_timed_;
  DataContainer *container_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

// static global object
MultiArrayPublisher *getMultiArrayPublisherPP()
{
  static MultiArrayPublisher x(getContainerObj(), "/dotnect_platform/point_pixel_postion", 17);
  return &x;
}

// static global object for felix
MultiArrayPublisher *getMultiArrayPublisherTS()
{
  static MultiArrayPublisher x(getContainerObj(), "/dotnect_platform/tele_speaker", 0);
  return &x;
}

// Function is called to visualize the android refrence frame_2
// The broadcaster sends transforms for the android refrence frame originating from the rotation-vector sensor
void findRefFrame2(const ros::TimerEvent&)
{
  ROS_WARN_THROTTLE(1,NNAME "timer on");
  // static vector<double> q2to0(7);
  //
  // if (getContainerObj()->checkVecFlag(2, 11) == 0)
  // {
  //   ROS_WARN_THROTTLE(5,NNAME ": empty /dotnect_dgram_socket/rot. Retrying... (would use identity q)");
  //   q2to0[0] = ros::Time::now().toNSec();  // implicit double(ulong)
  //   q2to0[1] = 18;
  //   q2to0[2] = 20;  // quaternion transforms coordinates from frame_2 to frame_0
  //   q2to0[6] = 1;
  //   getContainerObj()->setVec(q2to0);
  //   return;
  // }
  // ROS_INFO_ONCE("Superpose smartphone-frame with the frame you want as real-world refrence.");
  // ROS_INFO_ONCE("The real-world-refrence frame in digital RViz is the robot base frame_0.");
  // ROS_INFO_ONCE("If you hit the real-world-refrence frame later again, the relation in RViz of frame_3 to frame_0 will "
  //               "be the identity I = δij.");
  // ROS_INFO_ONCE("Info on smartphone-fixed frame "
  //               "https://developer.android.com/guide/topics/sensors/sensors_overview.html#sensors-coords");
  // // ROS_INFO_ONCE("Info on android-refrence frame_2
  // // https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-rotate");
  // ROS_INFO_THROTTLE(10, "Superpose, then tick off right-click menu of interactive marker when frame_2 stopped "
  //                       "drifting.");
  //
  // vector<double> rot;
  // getContainerObj()->getVec(2, 11, rot);
  // tf::Transform tf(tf::Quaternion(rot[3], rot[4], rot[5], rot[6]), tf::Vector3(0, 0, 1));
  // tf::TransformBroadcaster br;
  // br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "frame_0", "frame_2"));
  //
  // q2to0[0] = ros::Time::now().toNSec();  // implicit double(ulong) conversion
  // q2to0[1] = 18;
  // q2to0[2] = 20;
  // q2to0[3] = rot[3];
  // q2to0[4] = rot[4];
  // q2to0[5] = rot[5];
  // q2to0[6] = rot[6];
  // getContainerObj()->setVec(q2to0);
}

// publish dynamic and static RViz markers
class MarkerArrayPublisher
{
public:
  MarkerArrayPublisher(DataContainer *container) : container_(container)
  {
    // point_f1    pos:3x, ori:4, scale:3, color:4, type:1, action:1, duration:2 id:1;
    double arr[19] = { 0, 0, 0, 0, 0, 0, 1, 0.05, 0.05, 0.05, 0.6, 0.6, 0.6, 1, 2, 0, 1, 0, 0 };
    makeMarker("frame_1", arr);
    marker_array_.markers.push_back(marker_);

    show_urdf_ = container->getInt(4);
    if (show_urdf_)
    {
      // smartphone
      double arr[19] = { 0, 0, 0, 0, 0, 0, 1, 0.17, 0.35, 0.018, 0.5, 0.5, 0.5, 0.95, 1, 0, 1, 0, 0 };
      makeMarker("frame_3", arr);
      marker_array_.markers.push_back(marker_);
    }
    else
    {
      // green ball is the initial/current end-effector position
      double arr[19] = { 0, 0, 0, 0, 0, 0, 1, 0.05, 0.05, 0.05, 0, 1, 0, 1, 2, 0, 1, 0, 0 };
      arr[0] = container_->getInt(5);
      arr[1] = container_->getInt(6);
      arr[2] = container_->getInt(7);
      makeMarker("frame_0", arr);
      marker_array_.markers.push_back(marker_);
      // red ball
      double arr2[19] = { 0, 0, 0, 0, 0, 0, 1, 0.05, 0.05, 0.05, 1, 0, 0, 1, 2, 2, 1, 0, 1 };
      makeMarker("frame_0", arr2);
      marker_array_.markers.push_back(marker_);
      // maximum jump range sphere
      double arr3[19] = { 0, 0, 0, 0, 0, 0, 1, 0.5, 0.5, 0.5, 1, 0, 0, 0.5, 2, 2, 1, 0, 2 };
      makeMarker("frame_0", arr3);
      marker_array_.markers.push_back(marker_);
    }
    // advertise marker array
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dotnect_platform/MarkerArrayPublisher", 3);
  }
  ~MarkerArrayPublisher()
  {
  }
  // initalize marker for rviz
  void makeMarker(basic_string<char> frame_id, double *vars)
  {
    marker_.header.frame_id = frame_id;
    marker_.header.stamp = ros::Time::now();
    marker_.ns = NNAME;
    marker_.pose.position.x = vars[0];
    marker_.pose.position.y = vars[1];
    marker_.pose.position.z = vars[2];
    marker_.pose.orientation.y = vars[4];
    marker_.pose.orientation.z = vars[5];
    marker_.pose.orientation.w = vars[6];
    marker_.scale.x = vars[7];
    marker_.scale.y = vars[8];
    marker_.scale.z = vars[9];
    marker_.color.r = vars[10];
    marker_.color.g = vars[11];
    marker_.color.b = vars[12];
    marker_.color.a = vars[13];
    marker_.type = vars[14];
    marker_.action = vars[15];
    marker_.lifetime = ros::Duration(vars[16], vars[17]);
    marker_.id = vars[18];
  }
  void publishMarkerCallback(const ros::TimerEvent &)
  {
    boost::mutex::scoped_lock lock(mtx_);
    if (container_->checkVecFlag(0, 15))
    {
      container_->getVec(0, 15, point_f1_);
      marker_array_.markers[0].pose.position.x = point_f1_[3] * 1e-3;
      marker_array_.markers[0].pose.position.y = point_f1_[4] * 1e-3;
      marker_array_.markers[0].pose.position.z = point_f1_[5] * 1e-3;
    }
    if (show_urdf_)
    {
      // Add or fade redBall depending on container j:21
      if (container_->checkVecFlag(0, 21) == 1)
      {
        double discards = container_->getVecDbl(0, 21, 3);
        marker_array_.markers[2].action = discards;
        marker_array_.markers[3].action = discards;
      }
    }
    // update all timestamps and publish
    for (int i = 0; i < marker_array_.markers.size(); ++i)
    {
      marker_array_.markers[i].header.stamp = ros::Time::now();
    }
    pub_.publish(marker_array_);
  }

private:
  bool show_urdf_;
  std::vector<double> point_f1_;
  DataContainer *container_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  visualization_msgs::Marker marker_;
  visualization_msgs::MarkerArray marker_array_;
  boost::mutex mtx_;
};

// static global object
MarkerArrayPublisher *getMarkerArrayPub()
{
  static MarkerArrayPublisher x(getContainerObj());
  return &x;
}

// Regularly publish transforms, called by ros timer
// The tf pose updates are from the the latest interactive marker feedback
// incoming every time the user drag & drops an interactive marker
class TransformHandler
{
public:
  TransformHandler()
  {
    getContainerObj()->getVec(3, 11, v_q2to3_);
    getContainerObj()->getVec(3, 18, v_q2to0_);
    getContainerObj()->getVec(3, 16, point_f1_xma_);

    // default coordinate-transformation from frame_1 to frame_0
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(0, 0, 0));
    tf.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_1to0_ = tf::StampedTransform(tf, ros::Time::now(), "/frame_0", "/frame_1");
  }
  ~TransformHandler()
  {
  }
  void updateFeedback(...)
  {
    write_lock_ w_lock(mutex_shr_);
    // ...
  }
  // get dynamic transform for point from child to parent, no waitForTransform (just throw then)
  void getTransform(string parent, string child, tf::StampedTransform &tf_1to0_)
  {
    try
    {
      tf_li_.lookupTransform(parent, child, ros::Time(0), tf_1to0_);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN_THROTTLE(10, "%s Retrying...", ex.what());
    }
  }
  // publish frame_3 on the averaged point_f0_xma
  void publishTransform(const ros::TimerEvent &)
  {
    lock_shared_shared_ r_lock(mutex_shr_);
    if (getContainerObj()->checkVecFlag(3, 11) == 1)
    {
      getContainerObj()->getVec(3, 11, v_q2to3_);
    }
    if (getContainerObj()->checkVecFlag(3, 16) == 1)
    {
      getContainerObj()->getVec(3, 16, point_f1_xma_);
      printf("checkVecFlag(2, 16) %f %f %f\n", point_f1_xma_[3], point_f1_xma_[4], point_f1_xma_[5]);
    }
    if (getContainerObj()->checkVecFlag(3, 18) == 1)
    {
      getContainerObj()->getVec(3, 18, v_q2to0_);
    }
    // lookup coordinate-transformation from frame_1 to frame_0
    getTransform("/frame_0", "/frame_1", tf_1to0_);
    // vector to tf::Vector3
    tf::Vector3 point_f1_xma(point_f1_xma_[3], point_f1_xma_[4], point_f1_xma_[5]);
    // subtract transformed origin as vector is a relative quantity
    tf::Vector3 point_f0_xma(tf_1to0_ * point_f1_xma - tf_1to0_ * tf::Vector3(0, 0, 0));
    // coordinate-transformation from frame_2 to frame_0
    tf::Quaternion q2to0(v_q2to0_[3], v_q2to0_[4], v_q2to0_[5], v_q2to0_[6]);
    // coordinate-transformation from frame_2 to frame_3
    tf::Quaternion q2to3(v_q2to3_[3], v_q2to3_[4], v_q2to3_[5], v_q2to3_[6]);
    // coordinate-transformation from frame_3 to frame_0
    tf::Quaternion q3to0 = q2to0.inverse() * q2to3;
    // the coordinate-rotation with sendTransform from parent frame_0 to child frame_3
    // is equal to the coordinate-transformation quaternion q3to2
    tf::Transform tf(q3to0, point_f0_xma);
    tf_br_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "frame_0", "frame_3"));
    // trigger convert to val3 syntax that fits staeubli robot control and publish string
    // getGenerateVal3()->sendMovej(point_f0_xma.x(), point_f0_xma.y(), point_f0_xma.z(), q3to0.x(), q3to0.y(), q3to0.z(),
    getMultiArrayPublisherTS()->publish(point_f0_xma.x(), point_f0_xma.y(), point_f0_xma.z(), q3to0.x(), q3to0.y(), q3to0.z(),
                                           q3to0.w(), true, false);
  }

private:
  vector<double> point_f1_xma_;
  vector<double> v_q2to0_;
  vector<double> v_q2to3_;
  tf::StampedTransform tf_1to0_;
  tf::TransformListener tf_li_;
  tf::TransformBroadcaster tf_br_;
  boost::shared_mutex mutex_shr_;
  typedef boost::unique_lock<boost::shared_mutex> write_lock_;  // block new readers and wait until
  typedef boost::shared_lock<boost::shared_mutex> lock_shared_shared_;   // all lock_shared_shared_s are out of scope (released)
};

// static global object
TransformHandler *getTransformHandler()
{
  static TransformHandler x;
  return &x;
}

// pauses caller
bool customSleep(int sec, int nsec)
{
  struct timespec req, rem;
  req.tv_sec = sec;
  req.tv_nsec = nsec;
  if (nanosleep(&req, &rem) != 0)
  {
    printf(ACYELLOW NNAME ": nanosleep rem %d:%d strerror:%s" ACRESETN, (int)rem.tv_sec, (int)rem.tv_nsec, strerror(errno));
    return false;
  }
  return true;
}

// stack mod
void  setupTimer(){

  }

  struct CommaIterator
  :
    public std::iterator<std::output_iterator_tag, void, void, void, void>
  {
    std::ostream *os;
    std::string comma;
    bool first;

    CommaIterator(std::ostream& os, const std::string& comma)
    :
      os(&os), comma(comma), first(true)
    {
    }

    CommaIterator& operator++() { return *this; }
    CommaIterator& operator++(int) { return *this; }
    CommaIterator& operator*() { return *this; }
    template <class T>
    CommaIterator& operator=(const T& t) {
      if(first)
        first = false;
      else
        *os << comma;
      *os << t;
      return *this;
    }
  };

// process ui-client requests
bool processUiService(dotnect_custom_msgs::DotnectUi::Request &req, dotnect_custom_msgs::DotnectUi::Response &res) {
  if (req.object_name == "ui_validjumps") {

  } else if (req.object_name == "ui_rotation_offset") {
    if (req.rint) {
      *(getContainerObj()->getTimerPtr(2)) = getContainerObj()->getNodeHandle()->createTimer(ros::Duration(0.1), findRefFrame2);
      printf(NNAME ": timer findRefFrame2 started.\n");
    } else {
      getContainerObj()->getTimerPtr(2)->stop();
      printf(NNAME ": timer findRefFrame2 stopped.\n");
    }
  } else if (req.object_name == "ui_xma_change") {
    // stop thread
    getContainerObj()->setInt(14, 1);
    customSleep(1,0);
    // start again with new values
      getContainerObj()->setInt(1, req.rints[0]);
      // getContainerObj()->setInt(2, 0);
      getContainerObj()->setInt(3, req.rints[1]);
      *(getContainerObj()->getThreadXMA()) = boost::thread(MOVAV::moving_average, getContainerObj(), 15);
    }

  // print request TODO push to above with on off
  std::ostringstream oss_rbools;
  std::ostringstream oss_rints;
  std::ostringstream oss_rdbls;
  std::copy(req.rbools.begin(), req.rbools.end(), CommaIterator(oss_rbools, ","));
  std::copy(req.rints.begin(), req.rints.end(), CommaIterator(oss_rints, ","));
  std::copy(req.rdbls.begin(), req.rdbls.end(), CommaIterator(oss_rdbls, ","));
  printf(ACGREEN NNAME ": client request:\nobject_name:%s\nbool:%d\nbools:%s\nint:%d\nints:%s\nrdbl:%f\nrdbls:%s\nrstr:%s\n" ACRESETN, req.object_name.c_str(), req.rbool, oss_rbools.str().c_str(), req.rint, oss_rints.str().c_str(), req.rdbl, oss_rdbls.str().c_str(), req.rstr.c_str());

  // answer that request got sorted
  res.rbool = true;
  return true;
}

// main function processes user input, starts timers and threads
int main(int argc, char **argv)
{
  if (argc < 11)
  {
    printInfo(argv);
    exit(0);
  }
  // wait for roslaunch server to prevent nodes printf entangling (only reason)
  customSleep(3, 0);

  ros::init(argc, argv, NNAME);
  ros::NodeHandle nh;

  // printInfo
  int tmp;
  for (int i = 1; i < 11; ++i)
  {
    istringstream(argv[i]) >> tmp;
    getContainerObj()->setInt(i, tmp);
  }

  // add subscriptions
  printf(NNAME ": subscribes on: \n%s\n/dotnect_markers/menu_state_change\n", getSensorTopic(11).c_str());
  ros::Subscriber subContObj11 = nh.subscribe(getSensorTopic(11), 3, callbMAFloat64);
  ros::Subscriber subContObj19 = nh.subscribe("/dotnect_markers/menu_state_change", 3, callbInt32A);
  TrackingClass tracking_class(getContainerObj());
  tracking_class.subscribeImgSrc("/camera/depth/image_raw");

  // create timer to update marker array
  *(getContainerObj()->getTimerPtr(0)) = nh.createTimer(ros::Duration(0.1), &MarkerArrayPublisher::publishMarkerCallback, getMarkerArrayPub());
  // update transform
  *(getContainerObj()->getTimerPtr(1)) = nh.createTimer(ros::Duration(0.1), &TransformHandler::publishTransform, getTransformHandler());
  // flush data to callback functions
  boost::thread t_spinner(ros_spin);
  // start averaging point_f1 to point_f1_xma
  *(getContainerObj()->getThreadXMA()) = boost::thread(MOVAV::moving_average, getContainerObj(), 15);
  // react to incoming verbose signal
  ros::ServiceServer service = nh.advertiseService("dotnect_ui_service", processUiService);

  // shutdown handling
  t_spinner.join();
  printf(NNAME ": t_spinner terminated \n");
  getContainerObj()->setInt(14, 1);
  getContainerObj()->getThreadXMA()->join();
  printf(NNAME ": t_xma_average_ terminated \n");
  getContainerObj()->getTimerPtr(0)->stop();
  getContainerObj()->getTimerPtr(1)->stop();
  getContainerObj()->getTimerPtr(2)->stop();
  printf(NNAME ": timers stopped \n");
  printf(ACCYAN NNAME " returned" ACRESETN);
  return 0;
}
// EOF

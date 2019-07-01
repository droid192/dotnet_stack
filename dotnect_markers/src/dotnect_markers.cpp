// file: dotnect_markers.cpp, style: README.me
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// Add interactive markers to Rviz, so the environment and transforms are dynamic.
// The drag & drop fb_ of the markers is used to update pose of frame_0 and frame_1.
//
// NAME     DESCRIPTION
// see dotnect_platform.cpp
//
#include <stdio.h>
// boost
#include <boost/thread/mutex.hpp>
// ros
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
// local
#include "custom_defines.h"
#include "dotnect_custom_msgs/Int32Array.h"

#define NNAME "dotnect_markers"

namespace vm = visualization_msgs;
namespace im = interactive_markers;
using namespace std;

// global pointer
boost::shared_ptr<im::InteractiveMarkerServer> server;

// regularly publish transforms called by ros timer for frame_0 and frame_1,
// the tf pose-updates are from the marker fb_ triggered by user drag & drops
class TransformHandler
{
public:
  TransformHandler(string i_marker_name, vector<double> vars)
  {
    fb_.marker_name = i_marker_name;
    tf::Quaternion q(vars[1], vars[2], vars[3], vars[4]);
    tf::Vector3 v(vars[5], vars[6], vars[7]);
    tf::poseTFToMsg(tf::Pose(q, v), fb_.pose);
  }
  ~TransformHandler()
  {
  }
  void updateFeedback(const vm::InteractiveMarkerFeedbackConstPtr feedback_ptr)
  {
    u_lock_ w_lock(mutex_shr_);
    fb_ = *feedback_ptr;
  }
  void publishTransform(string frame_name)
  {
    s_lock_ r_lock(mutex_shr_);
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(fb_.pose.position.x, fb_.pose.position.y, fb_.pose.position.z));
    tf.setRotation(tf::Quaternion(fb_.pose.orientation.x, fb_.pose.orientation.y, fb_.pose.orientation.z,
                                  fb_.pose.orientation.w));
    br_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "dotnect_base_link", frame_name));
  }
private:
  boost::shared_mutex mutex_shr_;
  typedef boost::unique_lock<boost::shared_mutex> u_lock_;
  typedef boost::shared_lock<boost::shared_mutex> s_lock_;
  vm::InteractiveMarkerFeedback fb_;
  tf::TransformBroadcaster br_;
};

// Inital variables for the markers:
// scale_i_marker:1, q:4, p:3, scale_marker:3, color_marker:4, imode:1, type_marker:1
vector<double> getMarkerVariables(int marker_int)
{
  switch (marker_int) {
    case 0:
    {
      double vars[17] = {0.5, 0,0,0,1, -1,1,0, 0.4,0.4,0.06, 0.8,0.8,0.8,0.9, 9, 3 };
      return vector<double>(&vars[0], &vars[17]);
    }
    case 1:
    {
      double vars[17] = {0.5, -0.7071,0,0,0.7071, 1,1,1, 0.3,0.1,0.05, 0.8,0.8,0.8,0.9, 9, 3 };
      return vector<double>(&vars[0], &vars[17]);
    }
        default:
        printf(ACYELLOW NNAME": marker variabls name not available." ACRESETN);
        return vector<double>(0);
  }
}

// static global object
TransformHandler *getTransformHandlerFrame0()
{
  static TransformHandler x("frame_0", getMarkerVariables(0));
  return &x;
}

// static global object
TransformHandler *getTransformHandlerFrame1()
{
  static TransformHandler x("frame_1", getMarkerVariables(1));
  return &x;
}

// System timer refreshes transforms
void frameCallback(const ros::TimerEvent &)
{
  getTransformHandlerFrame0()->publishTransform("frame_0");
  getTransformHandlerFrame1()->publishTransform("frame_1");
}

// Publish signal int on topic set by marker menu
class Int32ArrayPublisher
{
public:
  Int32ArrayPublisher(string topic_name)
  {
    string topic = topic_name;
    pub_ = nh_.advertise<dotnect_custom_msgs::Int32Array>(topic, 10);
  }
  ~Int32ArrayPublisher()
  {
  }
  void pubSignalInt(int sig)
  {
    boost::mutex::scoped_lock lock(mtx_);
    dotnect_custom_msgs::Int32Array msg;
    msg.data.push_back(0);
    msg.data.push_back(sig);
    pub_.publish(msg);
  }
  void pubSignalVector(vector<int> vec)
  {
    boost::mutex::scoped_lock lock(mtx_1_);
    vec.insert(vec.begin(), 0);
    dotnect_custom_msgs::Int32Array msg;
    msg.data = vec;
    pub_.publish(msg);
  }
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  boost::mutex mtx_;
  boost::mutex mtx_1_;
};

// static global object, fully initalized when function returns
Int32ArrayPublisher *getInt32ArrayPublisher()
{
  static Int32ArrayPublisher x("/dotnect_markers/menu_state_change");
  return &x;
}

// static global object
im::MenuHandler *getMenuHandler()
{
  static im::MenuHandler x;
  return &x;
}

// Handle marker feedback to update frame_x pose
void processMarkerFeedback(const vm::InteractiveMarkerFeedbackConstPtr &fb_Ptr)
{
  int eventType = fb_Ptr->event_type;
  string i_marker_name = fb_Ptr->marker_name;
  im::MenuHandler::EntryHandle handle = fb_Ptr->menu_entry_id;
  im::MenuHandler::CheckState state;
  getMenuHandler()->getCheckState(handle, state);
  printf(NNAME ": marker %s on handle %d now state %d\n", i_marker_name.c_str(),
         handle, state);
  fflush(stdout);

  if (eventType == 0)
  {  // KEEP_ALIVE
    // sent while dragging to keep up control of the marker
  }
  else if (eventType == 1 && i_marker_name == "marker_0")
  {  // POSE_UPDATE
    getTransformHandlerFrame0()->updateFeedback(fb_Ptr);
  }
  else if (eventType == 1 && i_marker_name == "marker_1")
  {  // POSE_UPDATE
    getTransformHandlerFrame1()->updateFeedback(fb_Ptr);
  }
  else if (eventType == 2)
  {  // MENU_SELECT
    if (state == im::MenuHandler::NO_CHECKBOX)
    {  // if state:0
    }
    else if (state == im::MenuHandler::UNCHECKED)
    {  // if state:1
      getMenuHandler()->setCheckState(handle, im::MenuHandler::CHECKED);
    }
    else if (state == im::MenuHandler::CHECKED)
    {  // if state:2
      getMenuHandler()->setCheckState(handle, im::MenuHandler::UNCHECKED);
    }
    // publish current menu state
    vector<int> states;
    for (int handle = 1; handle < 3; handle++)
    {
      getMenuHandler()->getCheckState(handle, state);
      states.push_back(handle);
      states.push_back(state);
    }
    getInt32ArrayPublisher()->pubSignalVector(states);
    getMenuHandler()->reApply(*server);
    server->applyChanges();
  }
  else if (eventType == 3)
  {  // BUTTON_CLICK
    // ROS_INFO_STREAM( ":button click at...");
  }
  else if (eventType == 4)
  {  // MOUSE_DOWN
  }
  else if (eventType == 5)
  {  // MOUSE_UP
  }
}

// Create marker with full 6-DOF freedom (trans. & rot.)
// Each marker has two controls:
// i_marker = (noni_control + plain_marker) + i_control
void make6DofMarker(vm::InteractiveMarker &i_marker, const string name, vector<double> vars)
{
  i_marker.name = name;
  i_marker.description = "6DOF_MOVE_ROTATE";
  i_marker.header.frame_id = "dotnect_base_link";
  i_marker.header.stamp = ros::Time::now();
  i_marker.scale = vars[0];
  i_marker.pose.orientation.x = vars[1];
  i_marker.pose.orientation.y = vars[2];
  i_marker.pose.orientation.z = vars[3];
  i_marker.pose.orientation.w = vars[4];
  i_marker.pose.position.x = vars[5];
  i_marker.pose.position.y = vars[6];
  i_marker.pose.position.z = vars[7];

  // non-interactive control
  // wiki.ros.org/rviz/DisplayTypes/Marker
  vm::InteractiveMarkerControl noni_control;
  noni_control.always_visible = true;
  vm::Marker plain_marker;
  plain_marker.type = vars[16];
  plain_marker.scale.x = vars[8];
  plain_marker.scale.y = vars[9];
  plain_marker.scale.z = vars[10];
  plain_marker.color.r = vars[11];
  plain_marker.color.g = vars[12];
  plain_marker.color.b = vars[13];
  plain_marker.color.a = vars[14];
  noni_control.markers.push_back(plain_marker);
  i_marker.controls.push_back(noni_control);

  i_marker.controls[0].interaction_mode = vars[15];

  // interactive control
  vm::InteractiveMarkerControl i_control;
  i_control.orientation_mode = vm::InteractiveMarkerControl::FIXED;
  quaternionTFToMsg(tf::Quaternion(1, 0, 0, 1).normalize(), i_control.orientation);  // qx qy qz qw
  i_control.name = "rx";
  i_control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
  i_marker.controls.push_back(i_control);
  i_control.name = "mx";
  i_control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  i_marker.controls.push_back(i_control);

  quaternionTFToMsg(tf::Quaternion(0, 1, 0, 1).normalize(), i_control.orientation);
  i_control.name = "rz";
  i_control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
  i_marker.controls.push_back(i_control);
  i_control.name = "mz";
  i_control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  i_marker.controls.push_back(i_control);

  quaternionTFToMsg(tf::Quaternion(0, 0, 1, 1).normalize(), i_control.orientation);
  i_control.name = "ry";
  i_control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
  i_marker.controls.push_back(i_control);
  i_control.name = "my";
  i_control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  i_marker.controls.push_back(i_control);
}

// Function pauses caller
bool customSleep(int sec, int nsec)
{
  struct timespec req, rem;
  req.tv_sec = sec;
  req.tv_nsec = nsec;
  if (nanosleep(&req, &rem) != 0)
  {
    printf(ACYELLOW NNAME "rem %d:%d strerror:%s" ACRESETN, (int)rem.tv_sec, (int)rem.tv_nsec, strerror(errno));
    return false;
  }
  return true;
}


// main function sets up interactive marker server and creates the right-click menu
int main(int argc, char **argv)
{
  // wait for roslaunch server to prevent nodes printf entangling (only reason)
  customSleep(3, 0);

  ros::init(argc, argv, NNAME);
  ros::NodeHandle nh;

  // pointer to a dynamically allocated object
  server.reset(new im::InteractiveMarkerServer(NNAME, "", false));
  // wait till server is fully initalized
  ros::Duration(0.1).sleep();
  // create a timer to constantly publish transforms
  ros::Timer timer_frames = nh.createTimer(ros::Duration(0.1), frameCallback);

  // prepare and pump callbacks
  // create marker for robot base
  vm::InteractiveMarker marker_0;
  make6DofMarker(marker_0, "marker_0", getMarkerVariables(0));
  server->insert(marker_0);
  server->setCallback(marker_0.name, &processMarkerFeedback);
  // create marker for camera
  vm::InteractiveMarker marker_1;
  make6DofMarker(marker_1, "frame_1", getMarkerVariables(1));
  server->insert(marker_1);
  server->setCallback(marker_1.name, &processMarkerFeedback);
  // let changes take effect
  server->applyChanges();
  // apply right-click menu to interactive markers
  im::MenuHandler::EntryHandle eH1 = getMenuHandler()->insert("empty handle 1", &processMarkerFeedback);
  im::MenuHandler::EntryHandle eH2 = getMenuHandler()->insert("empty handle 2", &processMarkerFeedback);
  getMenuHandler()->setCheckState(eH1, im::MenuHandler::NO_CHECKBOX);
  getMenuHandler()->setCheckState(eH2, im::MenuHandler::UNCHECKED);
  // let changes take effect
  getMenuHandler()->apply(*server, marker_0.name);
  getMenuHandler()->apply(*server, marker_1.name);
  // menu signal publisher
  getInt32ArrayPublisher();

  // pump callbacks
  ros::spin();
  timer_frames.stop();
  printf(NNAME": timer_frames stopped\n");
  // delete object by deleting last (and only) pointer
  server.reset();
  printf(NNAME" returned.\n");
  return 0;
}
// EOF

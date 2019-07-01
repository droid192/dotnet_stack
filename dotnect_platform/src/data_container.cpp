// file: data_container.cpp, style: README.md
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// The class object stores data for thread [i] safe access.
// Each thread has its own list of flags [j].
// -> v_dbls_[j] and v_dbls_flags_[i][j]
//
//   [i]                        GET       SET
//    0: marker_array_publisher 21
//    1:moving_average          15,20     16,17
//    ??                         11,19    18
//    2:joints_publisher        18
//    3:transform_handler       11,16,18
//    4:pub_multi_array         17
//    -:interactive_marker_node -         19,20,21
//    -:via topic                         11,15,19,20
//   [j]:                 STRUCTURE                     INFO
//    0;-
//    1: acc              t,j,accuracy,val1-3           accelerometer sensor
//    2: mag              t,j,accuracy,val1-3           magnetometer sensor
//    3: orient           t,j,accuracy,val1-3           orientation sensor
//    4: gyro             t,j,accuracy,val1-3           gyroscope sensor
//    5: light                                          light sensor
//    6: pres                                           pressure sensor
//    7: temp,                                          temperature sensor
//    8: prox                                           proximity sensor
//    9: gravity                                        gravity sensor
//    10:linacc                                         linear accelerometer sensor (no grav)
//    11:rot              t,j,accuracy,val1-4           rotation vector sensor
//    12:humid                                          humidity sensor
//    13:ambtmp                                         ambient temperature sensor
//    14:-
//    15:point_f1          t,j,frameCount,val1-3         kinect_tracking point (frame_1, 3D, mm, t [s])
//    16:point_f1_xma       t,j,accuracy,val1-3           same point averaged [ns]
//    17:point_pxpos       t,j,u,v                       position of point_f1 in picture matrix
//    18:q_rf2tof0        t,j,02,val1-4                 quaternion that rotates frame_2 to frame_0
//    [k]
//     0:-
//     1:xma              0-2                           -> dotnect_platform:printInfo
//     2:posVel           0-3 TODO remove arg support
//     3:nPoints          0<x
//     4:showURDF         0-1
//     5:initEndEffX      x
//     6:initEndEffY      x
//     7:initEndEffZ      x
//     8:scaleX           x
//     9:scaleY           x
//    10:scaleZ           x
//    11:-
//    12:discards         0:ok                         indicates a current series of discarded points
//                        1:discards
//    13:ui_validjumps     0:off                        .ui element, also enables jump range markers for user guidance
//                        1:on
//    14:ui_xma_change    0:no change
//                        1:changed
//    [t] timers
//     0:MarkerArrayPublisher::publishMarkerCallback
//     1:TransformHandler::publishTransform
//     2:findRefFrame2
//
#include <stdio.h>
#include <vector>
// boost
#include <boost/thread/mutex.hpp>
// ros
#include <ros/time.h>
#include <ros/timer.h>
// local
#include "custom_defines.h"
#include "data_container.h"

DataContainer::DataContainer()
{
  // v_ints_ indicates certain states e.g. menu is checked or not
  v_ints_.resize(15, 0);
  // 1-10 are set by user arguments right after startup TODO what
  v_ints_[11] = 2;
  v_ints_[12] = 0;
  v_ints_[13] = 2;
  // v_dbls_ comprises all sensor related data
  // initialize with empty vector of biggest size in j
  std::vector<double> def(7, 0);
  def[0] = ros::Time::now().toNSec();
  v_dbls_.resize(19, def);
  for (int i = 0; i < v_dbls_.size(); ++i)
  {
    v_dbls_[i][1] = i;
  }
  // set defaults
  v_dbls_[11][6] = 1;
  v_dbls_[15][5] = 1;
  v_dbls_[16][5] = 1;
  v_dbls_[18][6] = 1;
  // down all v_dbls_flags_
  v_dbls_flags_.resize(5);
  for (int i = 0; i < 5; ++i)
  {
    v_dbls_flags_[i].resize(19, 0);
  }
  // prepare timers and threads
  v_timers_.resize(3, ros::Timer());
}

// member functions
int DataContainer::getInt(int k)
{
  if (checkArgsInts(k))
  {
    lock_shared_shared_(mtx_shr_ints_);
    return v_ints_[k];
  }
  return 0;
}
void DataContainer::setInt(int k, int value)
{
  if (checkArgsInts(k))
  {
    boost::unique_lock<boost::shared_mutex> WriteLock(mtx_shr_ints_);
    v_ints_[k] = value;
  }
  return;
}
bool DataContainer::checkArgsInts(uint k)
{
  if (k < v_ints_.size())
  {
    return true;
  }
  printf(ACYELLOW NNAME " checkArgsInts: bad k %d" ACRESETN, k);
  return false;
}
void DataContainer::setVec(const std::vector<double>& v_in)
{
  // only one thread can enter at once
  boost::upgrade_lock<boost::shared_mutex> lock(mtx_shr_dbls_);
  int j = v_in[1];
  if (checkArgsDbls(j))
  {
    // to start writing readers need to be blocked
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    v_dbls_[j] = v_in;
    for (int i = 0; i < 5; ++i)
    {
      v_dbls_flags_[i][j] = 1;
    }
  }
  return;
}
void DataContainer::getVec(int i, int j, std::vector<double>& v_out)
{
  if (checkArgsDbls(i, j))
  {
    lock_shared_shared_(mtx_shr_dbls_);
    v_out = v_dbls_[j];
    v_dbls_flags_[i][j] = 0;
    return;
  }
}
double DataContainer::getVecDbl(int i, int j, int l)
{
  if (checkArgsDbls(i, j, l))
  {
    lock_shared_shared_(mtx_shr_dbls_);
    v_dbls_flags_[i][j] = 0;
    return v_dbls_[j][l];
  }
  return 0.;
}
int DataContainer::checkVecFlag(int i, int j)
{
  if (checkArgsDbls(i, j))
  {
    lock_shared_shared_(mtx_shr_dbls_);
    return v_dbls_flags_[i][j];
  }
  return 0;
}
ros::Timer* DataContainer::getTimerPtr(int t){
        return &v_timers_[t];
}
ros::NodeHandle* DataContainer::getNodeHandle(){
  return &nh_;
}
boost::thread* DataContainer::getThreadXMA(){
  return &t_xma_average_;
}

// privates
bool DataContainer::checkArgsDbls(uint j)
{
  if (j < v_dbls_.size())
  {
    return true;
  }
  printf(ACYELLOW NNAME " checkArgsDbls: rejected j %d" ACRESETN, j);
  return false;
}
bool DataContainer::checkArgsDbls(uint i, uint j)
{
  if (i < v_dbls_flags_.size() || j < v_dbls_.size())
  {
    return true;
  }
  printf(ACYELLOW NNAME " checkArgsDbls: rejected i %d j %d" ACRESETN, i, j);
  return false;
}
bool DataContainer::checkArgsDbls(uint i, uint j, uint l)
{
  if (i < v_dbls_flags_.size() || j < v_dbls_.size() || l < v_dbls_[j].size())
  {
    return true;
  }
  printf(ACYELLOW NNAME " checkArgsDbls: rejected i %d j %d l %d" ACRESETN, i, j, l);
    return false;
}
bool DataContainer::customSleep(int sec, int nsec)
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
// EOF

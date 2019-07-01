// file: data_container.h, style: README.md
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// Info in DataContainer.cpp
//
#ifndef DATA_CONTAINER_H
#define DATA_CONTAINER_H

#include <vector>
// boost
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>
// ros
#include <ros/ros.h>
#include <ros/timer.h>

class DataContainer
{
private:
  typedef boost::shared_lock<boost::shared_mutex> lock_shared_shared_;
public:
  DataContainer();
  // for v_ints_ access
  int getInt(int index);
  void setInt(int index, int value);
  // for v_dbls_ access
  void setVec(const std::vector<double>& v_in);
  void getVec(int accessor, int typ, std::vector<double>& v_out);
  double getVecDbl(int accessor, int typ, int index);
  int checkVecFlag(int accessor, int typ);
  // for timer start/stop
  ros::Timer* getTimerPtr(int t);
  ros::NodeHandle* getNodeHandle();
  // for thread t_xma_average_
  boost::thread* getThreadXMA();

private:
  bool checkArgsInts(uint index);
  bool checkArgsDbls(uint typ);
  bool checkArgsDbls(uint accessor, uint typ);
  bool checkArgsDbls(uint i, uint j, uint l);
  bool customSleep(int sec, int nsec);
  // flieds
  std::vector<int> v_ints_;
  std::vector<std::vector<double> > v_dbls_;
  std::vector<std::vector<int> > v_dbls_flags_;
  std::vector<ros::Timer> v_timers_;
  boost::thread t_xma_average_;
  boost::shared_mutex mtx_shr_ints_;
  boost::shared_mutex mtx_shr_dbls_;
  ros::NodeHandle nh_;
};
#endif
// EOF

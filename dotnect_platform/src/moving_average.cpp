// file: moving_average.cpp, style: README.md
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// Use moving averages (SMA, LMA and EMA) to calculate averages from up to 3 vector entries
// SMA: simple moving average
// LMA: linear moving average
// EMA: exponential moving average
//
#include <stdio.h>
#include <cmath>
#include <vector>
// ros
#include <ros/console.h>
#include <ros/ros.h>
// local
#include "data_container.h"
#include "moving_average.h"
#include "custom_defines.h"


namespace MOVAV
{
// verfiy input: does vector meet min/max jump step
bool validPoint(DataContainer * container, std::vector<double>& v_new, std::vector<double>& v_before)
{
  ROS_WARN("new avg\n" );
  fflush(stdout);
  // no validation
  if (container->getInt(13) == 0) {
    return true;
  }
  // otherwise calculate distances:
  // minimum distance to previous steering point, jump range = [min,max]
  double min = 10;
  // maximum ...
  double max = 500;
  // distance between new and last point
  double distance =
      sqrt((v_new[3] - v_before[3]) * (v_new[3] - v_before[3]) + (v_new[4] - v_before[4]) * (v_new[4] - v_before[4]) +
           (v_new[5] - v_before[5]) * (v_new[5] - v_before[5]));

    if (distance < min)
    {
      // endpoint arrived, no micro-adjustments below min
      ROS_DEBUG_THROTTLE(1, NNAME ": below jump range     %f < %f [mm]", distance, min);
      return false;
    }
    else if (distance > max)
    {
      // distance exceeds max, thus is out of jump range
      ROS_WARN_THROTTLE(1, NNAME ": exceeding jump range %f < %f [mm]", distance, max);
      return false;
    }
      // otherwise all ok
      return true;
}

// moving average on vector nPoints a,(b,(c)) of given sensor, thread safe
// container: unique class object
// xma:      average type
// pv:       use xma to smooth position or velocity
// nPoints:  window length, longer window equals smoother results, but slower end-point convergence
//              estimate: nPoints = freq *  w_time; [s = 1/s * s]
// initEndEff:  user set start point
void moving_average(DataContainer* container, int sensor_in)
{
  // xma average type
  int xma = container->getInt(1);
  int pv = container->getInt(2);
  int nPoints = container->getInt(3);
  // tell container in getVec which flag to strike
  int accessor;
  // to smooth vector entries
  int a = 0;
  int b = 0;
  int c = 0;
  // interactive marker menu 1:notchecked 2:checked
  double ui_validjumps = 1;
  // iteration counter
  int count = 1;
  // invalid iteration counter
  int invalidCount = 0;
  // time between last saved window-entry
  double t_delta = 0.;
  // smoothed entries
  double x = 0.;
  double y = 0.;
  double z = 0.;
  // velocity is calculated by the last two smoothed entries divided by their t_delta
  // Suffix _delta indicates incremental steps.
  double v_x = 0.;
  double v_y = 0.;
  double v_z = 0.;
  double x_delta = 0.;
  double y_delta = 0.;
  double z_delta = 0.;
  double v_x_delta = 0.;
  double v_y_delta = 0.;
  double v_z_delta = 0.;
  // EMA adjusting factor
  double alpha;
  // XMA weights
  std::vector<double> weights;
  // temporary for setVec, p:3,4,5 v:6,7,8
  std::vector<double> vec_out(9, 0);
  // first t_delta will therefore be huge
  vec_out[0] = ros::Time::now().toNSec();
  // info on how many points were involved
  vec_out[2] = nPoints;
  // temporary vector for getVec
  std::vector<double> tmp(6, 0);
  // intital end-effector position
  std::vector<double> initEndEff(6, 0);
  // window holds past nPoints, w[length-1] is the oldest entry
  std::vector<std::vector<double> > w;
  // timespan before rechecking Flag
  struct timespec req, rem;
  req.tv_sec = 0;
  req.tv_nsec = 30000000;  // 33Hz

  if (sensor_in == 15)
  {  // 3D point
    accessor = 1;
    a = 3;
    b = 4;
    c = 5;
    initEndEff[0] = ros::Time::now().toNSec();
    initEndEff[1] = sensor_in;
    initEndEff[2] = nPoints;
    initEndEff[3] = container->getInt(5);
    initEndEff[4] = container->getInt(6);
    initEndEff[5] = container->getInt(7);
    for (int i = 0; i < nPoints; ++i)
    {
      w.push_back(initEndEff);
    }
    vec_out[1] = 16;
  }
  else
  {
    ROS_ERROR(NNAME": xma for Sensor_in %d not set up.", sensor_in);
    return;
  }

  // calculate weights
  weights.resize(nPoints, 1. / nPoints);  // SMA
  if (xma == 1)
  {  // LMA
    double sum = 0.;
    for (int i = 1; i <= nPoints; ++i)
    {
      weights[nPoints - i] = 1. / nPoints * i;
      sum += 1. / nPoints * i;
    }
    // normalize, division gets superfluous
    for (int i = 0; i < nPoints; ++i)
      weights[i] /= sum;
  }
  else if (xma == 2)
  {  // EMA
    // choose alpha (0;1), close to 1: aggressive response
    // 0: stay on init value
    // 1: output the input
    alpha = nPoints/100;
  }

  while (!ros::isShuttingDown() && container->getInt(14) == 0)
  {
    // do until valid vector
    bool valid = false;
    while (!valid)
    {
      if (container->checkVecFlag(accessor, sensor_in) == 1)
      {
        container->getVec(accessor, sensor_in, tmp);

        if (validPoint(container, tmp, w[0]))
        {
          valid = true;
          w.insert(w.begin(), 1, tmp);
          w.pop_back();
          container->setInt(12, 0);
          invalidCount = 0;
        }
        else if (invalidCount > 90)
        {
          container->setInt(12, 1);
          invalidCount = 0;
        }
        else
        {
          invalidCount++;
        }
      }
      else
      {
        nanosleep(&req, &rem);
        if (ros::isShuttingDown()) {
          return;
        }
      }
    }

    if (xma < 2)
    {  // SMA LMA
      x = 0.;
      y = 0.;
      z = 0.;
      for (int i = 0; i < nPoints; ++i)
      {
        x += w[i][a] * weights[i];
        y += w[i][b] * weights[i];
        z += w[i][c] * weights[i];
      }
    }
    else
    {  // EMA
      x = alpha * w[0][a] + (1. - alpha) * vec_out[3];
      y = alpha * w[0][b] + (1. - alpha) * vec_out[4];
      z = alpha * w[0][c] + (1. - alpha) * vec_out[5];
    }

    // calculate deltas and (velocity @EXPERIMENTAL), change!
    if (accessor == 1)
      t_delta = (w[0][0] - vec_out[0]);  // kinect uses secs
    else
      t_delta = (w[0][0] - vec_out[0]) * 1e-9;  // android nsces to secs

    if (pv > 0)
    {
      x_delta = x - vec_out[3];
      y_delta = y - vec_out[4];
      z_delta = z - vec_out[5];
      if (pv > 1)
      {
        v_x = x_delta / t_delta;
        v_y = y_delta / t_delta;
        v_z = z_delta / t_delta;
        if (pv > 2 && count > 0)
        {
          v_x_delta = (v_x - vec_out[6]);
          v_y_delta = (v_y - vec_out[7]);
          v_z_delta = (v_z - vec_out[8]);
        }
      }
    }

    // refresh outgoing vector
    vec_out[0] = w[0][0];
    if (pv == 0)
    {
      vec_out[3] = x;
      vec_out[4] = y;
      vec_out[5] = z;
    }
    else if (pv == 1)
    {
      vec_out[3] = x_delta;
      vec_out[4] = y_delta;
      vec_out[5] = z_delta;
    }
    else if (pv == 2)
    {
      vec_out[3] = v_x;
      vec_out[4] = v_y;
      vec_out[5] = v_z;
    }
    else if (pv == 3 && count > 0)
    {
      vec_out[3] = v_x_delta;
      vec_out[4] = v_y_delta;
      vec_out[5] = v_z_delta;
    }
    container->setVec(vec_out);

    // hold on to data for next loop
    if (pv > 0)
    {
      vec_out[3] = x;
      vec_out[4] = y;
      vec_out[5] = z;
      if (pv > 2)
      {
        vec_out[6] = v_x;
        vec_out[7] = v_y;
        vec_out[8] = v_z;
      }
    }
    count++;
  }
}
}
// EOF

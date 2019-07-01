// file: val3_fstream_socket.h, style: README.md
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// Serverside robot control handling for staeubli 6DOF
// generate, send and receive functions for stream socket handling
//
// preprocessor directive
#ifndef VAL3_FSTREAM_SOCKET_H
#define VAL3_FSTREAM_SOCKET_H
// std
#include <math.h>
#include <sys/socket.h>
#include <string>
#include <vector>
// ros
#include <ros/ros.h>
#include <tf/tf.h>

class Val3FstreamSocket
{
private:
  struct mDesc
  {
    int accel;          // maximum permitted joint acceleration, % of the robots nominal acceleration
    int vel;            // ... speed
    int decel;          // ... deceleration
    int tvel;           // max permitted translationsl speed of tcp [mm/s]
    int rvel;           // max permitted rotational sped of tcp [°/s]
    std::string blend;  // off, joint or Cartesian
    int leave;          // if joint or Cartesian: distance to target point to start blending
    int reach;          // if joint or Cartesian: distance after target point to stop blending
  } mDesci;
  struct config
  {
    int shoulder;  // righty, lefty, ssame, sfree
    int elbow;     // epositive, enegative, esame, efree
    int wrist;     // wpositive, wnegative, wsame, wfree
  } configi;

public:
  Val3FstreamSocket();
  ~Val3FstreamSocket();
  void manageSocket(int i);
  void setupSocket(const char *control_ip4, int port);
  void prepareCS8C();
  void genmDesc(mDesc &m, int i);
  void genConfig(config &c, int i);
  void quat2eul(double &qx, double &qy, double &qz, double &qw);
  void sendMovej(double x, double y, double z, double qx, double qy, double qz, double qw);
  void sendMJ(std::vector<double> j);
  void sendString(std::string str, bool read_buffer);
  std::vector<double> getJPos();
  std::string readFstreamBuffer();
private:
  int fd_;
  static const int buffer_size_ = 4096;
};
#endif
// EOF

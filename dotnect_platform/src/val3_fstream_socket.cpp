// file: val3_fstream_socket.cpp, style: README.md
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// assembles string that flows over topic to robot control in VAL3-format;
//
// C++11: snprintf, return values are moved
//
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <signal.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
// boost
#include <boost/algorithm/string.hpp>
// ros
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
// local
#include "val3_fstream_socket.h"
#include "custom_defines.h"

// constructor
Val3FstreamSocket::Val3FstreamSocket()
{
}

// close socket upon destruction
Val3FstreamSocket::~Val3FstreamSocket()
{
  manageSocket(0);
}

// pre-set socket configurations to setupSocket()
void Val3FstreamSocket::manageSocket(int i)
{
  // Roboter ip : 10.10.5.11
  // Roboter ip : 131.188.112.73
  // Roboter port 5653 (selm 10210)
  // Roboter Emulator port 5660
  // Pentagon:    131.188.112.158
  // Blackjack:   131.188.112.46
  // Lupus:       131.188.112.159
  // Lux (ubu.):  131.188.112.146
  if (i == 0)
  {
    close(fd_);
  }
  else if (i == 22)
  {
    // connect to robots CS8C
    if (fcntl(fd_, F_GETFD) != -1 || errno != EBADF)
    {
      close(fd_);
    }
    // CS8C J205
    Val3FstreamSocket::setupSocket("10.10.5.11", 5653);
  }
  else if (i == 32)
  {
    // connect to emulator
    if (fcntl(fd_, F_GETFD) != -1 || errno != EBADF)
    {
      close(fd_);
    }
    // CS8C J204
    Val3FstreamSocket::setupSocket("131.188.112.146", 5660);
  }
}

// connect to partner socket
void Val3FstreamSocket::setupSocket(const char *control_ip4, int port)
{
  // stream socket: p2p, lossless, ordered
  fd_ = socket(PF_INET, SOCK_STREAM, 0);
  if (fd_ == -1)
  {
    printf(ACRED NNAME ": setupSocket: %s" ACRESETN, strerror(errno));
    return;
  }
  // specific interface [optional]
  // char *devname = "enp9s0";
  // if (setsockopt(fd_, SOL_SOCKET, SO_BINDTODEVICE, devname, strlen(devname)) == -1) {
  //   printf(ACRED "platform_node: %f" ACRESETN, strerror(errno));
  //   return;
  // }
  // info struct on the server
  struct hostent *host;
  struct sockaddr_in addr;
  host = gethostbyname(control_ip4);
  if (host == 0)
  {
    printf(ACRED NNAME ": setupSocket: %s" ACRESETN, strerror(errno));
    return;
  }
  // initialize struct members
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr = *(struct in_addr *)host->h_addr;
  printf(NNAME ": setupSocket: connecting to %s:%d", inet_ntoa(addr.sin_addr), port);
  if (connect(fd_, (struct sockaddr *)&addr, sizeof(addr)) == -1)
  {
    printf(ACRED NNAME ": setupSocket: %s" ACRESETN, strerror(errno));
    return;
  }
  // output socket is connected
  printf(NNAME ": setupSocket: fstream_socket connected to %s:%d", inet_ntoa(addr.sin_addr), port);
  prepareCS8C();
}

// send keyword and motion descriptor to CS8C
void Val3FstreamSocket::prepareCS8C()
{
  char buffer[buffer_size_];
  int n = snprintf(buffer, sizeof(buffer), "enable");
  // truncation check
  if (buffer_size_ <= n || n < 0)
  {
    printf(ACRED NNAME ": prepareCS8C: %s" ACRESETN, strerror(errno));
  }
  if (send(fd_, buffer, strlen(buffer), 0) == -1)
  {
    printf(ACRED NNAME ": prepareCS8C: %s" ACRESETN, strerror(errno));
    return;
  }
  printf(NNAME ": prepareCS8C: %s\n", readFstreamBuffer().c_str());
  // send motion descriptor
  int mDescType = 0;
  int configType = 0;
  genmDesc(mDesci, mDescType);
  genConfig(configi, configType);
  n = snprintf(buffer, sizeof(buffer), "mdesc(%i,%i,%i,%i,%i,%s,%i,%i)", mDesci.vel, mDesci.vel, mDesci.decel,
               mDesci.tvel, mDesci.rvel, mDesci.blend.c_str(), mDesci.leave, mDesci.reach);
  // truncation check
  if (buffer_size_ <= n || n < 0)
  {
    printf(ACRED NNAME ": prepareCS8C: %s" ACRESETN, strerror(errno));
  }
  if (send(fd_, buffer, strlen(buffer), 0) == -1)
  {
    printf(ACRED NNAME ": prepareCS8C: %s" ACRESETN, strerror(errno));
    return;
  }
  printf(NNAME ": prepareCS8C: read_bufferer: '%s'\n", readFstreamBuffer().c_str());
}

// initialize motion descriptor
void Val3FstreamSocket::genmDesc(mDesc &m, int i)
{
  switch (i)
  {
    default:
      m.accel = 100;
      m.vel = 100;
      m.decel = 100;
      m.tvel = 9999;
      m.rvel = 9999;
      m.blend = "joint";
      m.leave = 50;
      m.reach = 50;
      ROS_WARN(NNAME ": genmDesc: defaults");
  }
}

// initialize configuration
void Val3FstreamSocket::genConfig(config &c, int i)
{
  switch (i)
  {
    default:
      c.shoulder = 0;
      c.elbow = 0;
      c.wrist = 0;
      ROS_WARN(NNAME ": genConfig: defaults");
  }
}

// convert quaternion to euler angles of xyz sequence
// see github.com/ItsmeJulian/quat2eul
void Val3FstreamSocket::quat2eul(double &qx, double &qy, double &qz, double &qw)
{
  double psi = atan2(2 * (qx * qw - qy * qz), (qw * qw - qx * qx - qy * qy + qz * qz));
  double theta = asin(2 * (qx * qz + qy * qw));
  double phi = atan2(2 * (qz * qw - qx * qy), (qw * qw + qx * qx - qy * qy - qz * qz));
  qx = psi;
  qy = theta;
  qz = phi;
  qw = 0.;
}

void Val3FstreamSocket::sendMovej(double x, double y, double z, double qx, double qy, double qz, double qw)
{
  quat2eul(qx, qy, qz, qw);
  std::stringstream ss;
  ss << "movej(" << x * 1e3 << "," << y * 1e3 << "," << z * 1e3 << "," << qx << "," << qy << "," << qz << ","
     << configi.shoulder << "," << configi.elbow << "," << configi.wrist << "," << mDesci.vel << ":" << mDesci.accel
     << ":" << mDesci.decel << ":" << mDesci.tvel << ":" << mDesci.rvel << ":" << mDesci.blend << ":" << mDesci.leave
     << ":" << mDesci.reach << ")";
    printf(ACYELLOW NNAME ": sendMovej: %s" ACRESETN, ss.str().c_str());
}

// send joint angles to robot
void Val3FstreamSocket::sendMJ(std::vector<double> j)
{
  char buffer[buffer_size_];
  int n = snprintf(buffer, sizeof(buffer), "mJ(%f,%f,%f,%f,%f,%f, %i,%i,%i,%i,%i,%s,%i,%i)", j[0], j[1], j[2], j[3],
                   j[4], j[5], mDesci.vel, mDesci.vel, mDesci.decel, mDesci.tvel, mDesci.rvel, mDesci.blend.c_str(),
                   mDesci.leave, mDesci.reach);
  // truncation check
  if (buffer_size_ <= n || n < 0)
  {
    printf(ACRED NNAME ": sendMJ: %s" ACRESETN, strerror(errno));
  }
  if (send(fd_, buffer, strlen(buffer), 0) == -1)
  {
    printf(ACRED NNAME ": sendMJ: %s" ACRESETN, strerror(errno));
    return;
  }
  printf(NNAME": sendMJ: readFstreamBuffer: '%s'\n", readFstreamBuffer().c_str());
}

// send trivial string
void Val3FstreamSocket::sendString(std::string str, bool read_buffer)
{
  char buffer[buffer_size_];
  int n = snprintf(buffer, sizeof(buffer), "%s", str.c_str());
  // truncation check
  if (buffer_size_ <= n || n < 0)
  {
    printf(ACRED NNAME ": sendString: %s" ACRESETN, strerror(errno));
  }
  if (send(fd_, buffer, strlen(buffer), 0) == -1)
  {
    printf(ACRED NNAME ": sendString: %s" ACRESETN, strerror(errno));
    return;
  }
  if (read_buffer == true)
  {
    printf(NNAME ": sendString: readFstreamBuffer: '%s'\n", readFstreamBuffer().c_str());
  }
}

// get joint angles in rad from 6DOF-robot
std::vector<double> Val3FstreamSocket::getJPos()
{
  char buffer[buffer_size_];
  sendString("jpos", false);
  // process databuf
  std::string tmpBuf = readFstreamBuffer();
  std::vector<std::string> tmp;
  std::vector<double> joints;
  boost::split(tmp, tmpBuf, boost::is_any_of(","));
  joints.resize(tmp.size());
  int j = 0;
  std::vector<std::string>::const_iterator itr;
  for (itr = tmp.begin(); itr != tmp.end(); ++itr)
  {
    std::istringstream iss(*itr);
    iss >> joints[j];
    joints[j] *= (M_PI / 180);
    j++;
  }
  return joints;
}

// read incoming
std::string Val3FstreamSocket::readFstreamBuffer()
{
  char response[buffer_size_];
  int byteReceived = recv(fd_, response, sizeof(response), 0);
  if (byteReceived == -1)
  {
    printf(ACRED NNAME ": readFstreamBuffer: '%s'" ACRESETN, strerror(errno));
  }
  return response;
}

// EOF

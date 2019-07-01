// file: test_dgram_socket.cpp, style: README.me
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
#include <std_srvs/SetBool.h>
#include <cstdlib>
#include <sstream>
#include <iostream>
// ros
#include <ros/console.h>
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_dgram_socket");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("dotnect_dgram_socket_service");

  std_srvs::SetBool srv;
  srv.request.data = true;
  if (argc > 1)
  {
    bool tmp;
    std::istringstream(argv[1]) >> tmp;
    srv.request.data = tmp;
  }

  if (client.call(srv))
  {
    printf("srv.request.data: %s \nsrv.response.success: %s\nsrv.response.message: %s\n",
             srv.request.data ? "true" : "false", srv.response.success ? "true" : "false",
             srv.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service!");
    return 1;
  }
  return 0;
}

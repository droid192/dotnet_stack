// file: test_ui_signals.cpp, style: README.me
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
#include <cstdlib>
#include <sstream>
#include <iostream>
// ros
#include <ros/console.h>
#include "ros/ros.h"
// local
#include "custom_defines.h"
#include "dotnect_custom_msgs/DotnectUi.h"

// output user args
void printInfo()
{
  printf(ACCYAN "$ dotnect_dgram_socket <1:object_name> <i:values>" ACRESETN);
  printf("    i:values depends on the gui object tested\n");
}

// setup client and message
int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printInfo();
    exit(0);
  }
  // setup client
  ros::init(argc, argv, "test_ui_signals");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<dotnect_custom_msgs::DotnectUi>("dotnect_ui_service");

  // setup service message
  dotnect_custom_msgs::DotnectUi srv;
  srv.request.object_name = argv[1];

  if (srv.request.object_name == "ui_rotation_offset") {
    std::istringstream(argv[2]) >> srv.request.rint;
  } else if (srv.request.object_name == "ui_xma_change") {
    int tmp;
    std::istringstream(argv[2]) >> tmp;
    srv.request.rints.push_back(tmp);
    std::istringstream(argv[3]) >> tmp;
    srv.request.rints.push_back(tmp);

  } else {
    ROS_ERROR("not yet implemented\n");
    return 1;
  }


  if (client.call(srv))
  {
    printf("request: %s %d\nresponse: %d\n", srv.request.object_name.c_str(), srv.request.rint, srv.response.rbool);
  }
  else
  {
    ROS_ERROR("Failed to call service.");
    return 1;
  }
  return 0;
}
// EOF

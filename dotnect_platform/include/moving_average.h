// file: moving_average.h, style: README.md
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// Info in moving_average.cpp
//
#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <vector>
// local
#include "data_container.h"

namespace MOVAV
{
  bool validPoint(std::vector<double>& v_new, std::vector<double>& v_before, int count);
  void moving_average(DataContainer* container, int sensor_in);
}
#endif
// EOF

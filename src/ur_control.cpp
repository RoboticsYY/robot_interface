// Copyright (c) 2019 Intel Corporation. All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <robot_interface/ur_control.hpp>

std::string getLocalIPAccessibleFromHost(std::string &host)
{
  URStream stream(host, UR_RT_PORT);
  return stream.connect() ? stream.getIP() : std::string();
}

bool URControl::moveToTcpPose(double x, double y, double z, 
                          double alpha, double beta, double gamma, 
                          double vel, double acc)
{
  return true;
}

bool URControl::open(const double distance)
{
  return true;
}

bool URControl::close(const double distance)
{
  return true;
}

bool URControl::pick(double x, double y, double z, 
                double alpha, double beta, double gamma, 
                double vel, double acc, double vel_scale, double approach)
{
  return true;
}

bool URControl::place(double x, double y, double z, 
                  double alpha, double beta, double gamma,
                  double vel, double acc, double vel_scale, double retract)
{
  return true;
}

bool URControl::start()
{
  return true;
}


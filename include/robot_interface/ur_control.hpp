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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robot_interface/control_base.hpp>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/factory.h"
#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/ur/rt_state.h"
#include "ur_modern_driver/ur/state.h"

static const std::string IP_ADDR_ARG("~robot_ip_address");
static const std::string REVERSE_IP_ADDR_ARG("~reverse_ip_address");
static const std::string REVERSE_PORT_ARG("~reverse_port");
static const std::string ROS_CONTROL_ARG("~use_ros_control");
static const std::string LOW_BANDWIDTH_TRAJECTORY_FOLLOWER("~use_lowbandwidth_trajectory_follower");
static const std::string MAX_VEL_CHANGE_ARG("~max_vel_change");
static const std::string PREFIX_ARG("~prefix");
static const std::string BASE_FRAME_ARG("~base_frame");
static const std::string TOOL_FRAME_ARG("~tool_frame");
static const std::string TCP_LINK_ARG("~tcp_link");
static const std::string JOINT_NAMES_PARAM("hardware_interface/joints");
static const std::string SHUTDOWN_ON_DISCONNECT_ARG("~shutdown_on_disconnect");

static const std::vector<std::string> DEFAULT_JOINTS = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                         "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

static const int UR_SECONDARY_PORT = 30002;
static const int UR_RT_PORT = 30003;

struct ProgArgs
{
public:
  std::string host;
  std::string prefix;
  std::string base_frame;
  std::string tool_frame;
  std::string tcp_link;
  std::string reverse_ip_address;
  int32_t reverse_port;
  std::vector<std::string> joint_names;
  double max_acceleration;
  double max_velocity;
  double max_vel_change;
  bool use_ros_control;
  bool use_lowbandwidth_trajectory_follower;
  bool shutdown_on_disconnect;
};

std::string getLocalIPAccessibleFromHost(std::string &host);

class URControl: public ArmControlBase
{
public:
  URControl(const std::string node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ArmControlBase(node_name, options)
  {
  }

  ~URControl()
  {
  }

  virtual bool moveToTcpPose(double x, double y, double z, 
                             double alpha, double beta, double gamma, 
                             double vel, double acc);

  virtual bool open(const double distance);

  virtual bool close(const double distance);

  virtual bool pick(double x, double y, double z, 
                    double alpha, double beta, double gamma, 
                    double vel, double acc, double vel_scale, double approach);
  
  virtual bool place(double x, double y, double z, 
                     double alpha, double beta, double gamma,
                     double vel, double acc, double vel_scale, double retract);

  bool start();

private:

  
};
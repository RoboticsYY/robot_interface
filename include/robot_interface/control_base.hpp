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

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct TcpPose
{
public:
  double x, y, z;
  double alpha, beta, gamma;
};

class ArmControlBase: public rclcpp::Node
{
public:

  ArmControlBase(const std::string node_name, const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
  }

  ~ArmControlBase()
  {
  }

  virtual bool moveToTcpPose(double x, double y, double z, 
                             double alpha, double beta, double gamma, 
                             double vel, double acc) = 0;

  virtual bool moveToTcpPose(const Eigen::Isometry3d& pose, double vel, double acc);

  virtual bool open(const double distance = 0) = 0;

  virtual bool close(const double distance = 0) = 0;

  virtual bool pick(double x, double y, double z, 
                    double alpha, double beta, double gamma, 
                    double vel, double acc, double vel_scale, double approach);
  
  virtual bool place(double x, double y, double z, 
                     double alpha, double beta, double gamma,
                     double vel, double acc, double vel_scale, double retract);

  void toTcpPose(const geometry_msgs::msg::PoseStamped& pose_stamped, TcpPose& tcp_pose);

  void toTcpPose(const Eigen::Isometry3d& pose, TcpPose& tcp_pose);

protected:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::vector<std::string> joint_names_;
};
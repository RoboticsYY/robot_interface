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

/**
 * @file control_base.hpp
 * @author Yu Yan
 * @date 29 Sep 2019
 * @brief Native robot control interface for visual manipulation.
 *
 * This file contains the control interface template to make the visual grasping. The interface is used 
 * for the control between a PC and an industrial robot controller. Collision detection is not considered in this
 * interface. The specific interfaces are expected to be filled with the communication protocal of a specific 
 * industrial robot, which are usually provided by the industrial robot manufacturors. 
 */

#pragma once

#include <mutex>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @brief Data type to represent robot arm's end-effector pose in 3D cartesian space.
 */
struct TcpPose
{
  double x; /**< Translation along x axis. */
  double y; /**< Translation along y axis. */
  double z; /**< Translation along z axis. */
  double alpha; /**< Euler angle of the rotation along x axis. */
  double beta;  /**< Euler angle of the rotation along y axis. */
  double gamma; /**< Euler angle of the rotation along z axis. */
};

/**
 * @brief Robot arm control interface.
 */
class ArmControlBase: public rclcpp::Node
{
public:

  /**
   * @brief Constructor of class #ArmControlBase.
   * @param node_name The name of the ROS2 node.
   * @param options ROS2 node options.
   */
  ArmControlBase(const std::string node_name, const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    time_out_ = 15.0;
  }

  /**
   * @brief Default destructor of class #ArmControlBase.
   */
  ~ArmControlBase()
  {
  }

  /**
   * @brief Move the robot end-effector to a goal position and orientation in 3D Cartesian space.
   * @param x Goal position on X dimension.
   * @param y Goal position on Y dimension.
   * @param z Goal position on Z dimension.
   * @param alpha Goal rotation euler angle along X axis.
   * @param beta Goal rotation euler angle along Y axis.
   * @param gamma Goal rotation euler angle along Z axis.
   */
  virtual bool moveToTcpPose(double x, double y, double z, 
                             double alpha, double beta, double gamma, 
                             double vel, double acc) = 0;

  virtual bool moveToTcpPose(const Eigen::Isometry3d& pose, double vel, double acc);

  virtual bool open(const double distance = 0) = 0;

  virtual bool close(const double distance = 0) = 0;

  virtual bool pick(double x, double y, double z, 
                    double alpha, double beta, double gamma, 
                    double vel, double acc, double vel_scale, double approach);
  
  virtual bool pick(const geometry_msgs::msg::PoseStamped& pose_stamped, 
                    double vel, double acc, double vel_scale, double approach);

  virtual bool place(double x, double y, double z, 
                     double alpha, double beta, double gamma,
                     double vel, double acc, double vel_scale, double retract);

  virtual bool place(const geometry_msgs::msg::PoseStamped& pose_stamped,
                     double vel, double acc, double vel_scale, double retract);

  void toTcpPose(const geometry_msgs::msg::PoseStamped& pose_stamped, TcpPose& tcp_pose);

  void toTcpPose(const Eigen::Isometry3d& pose, TcpPose& tcp_pose);

  virtual bool checkTcpGoalArrived(Eigen::Isometry3d& tcp_goal);

protected:
  // Joint state publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  // Joint names
  std::vector<std::string> joint_names_;
  // Current tcp pose
  TcpPose tcp_pose_;
  // Mutex to guard the tcp_pose usage
  std::mutex m_;
  // Motion running duration timeout
  double time_out_;
};
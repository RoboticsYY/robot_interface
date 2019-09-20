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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::Rate rate(2);

  auto node = std::make_shared<URControl>("ur_test", rclcpp::NodeOptions());

  node->start();

  rate.sleep();

  node->moveToTcpPose(-0.350, -0.296, 0.145, 3.14159, 0, 0, 0.3, 0.3);

  rclcpp::spin(node);

  return 0;
}
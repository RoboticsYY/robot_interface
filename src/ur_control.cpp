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
  std::string command_script = "movej(p[" + 
                               std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "," +
                               std::to_string(alpha) + "," + std::to_string(beta) + "," + std::to_string(gamma) + "]," + 
                               std::to_string(vel) + "," + std::to_string(acc) + ")\n";
  urscriptInterface(command_script);
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

bool URControl::urscriptInterface(const std::string command_script)
{
  bool res = rt_commander_->uploadProg(command_script);
  if (!res)
  {
    LOG_ERROR("Program upload failed!");
  }

  return res;
}

bool URControl::start()
{
  args_.host = "192.168.0.5";
  args_.reverse_ip_address = "";
  args_.reverse_port =5001 ;
  args_.max_vel_change = 15.0;
  args_.max_velocity = 10.0;
  args_.joint_names = DEFAULT_JOINTS;
  args_.shutdown_on_disconnect = true;

  local_ip_ = args_.reverse_ip_address;

  // if no reverse IP address has been configured, try to detect one
  if (local_ip_.empty())
  {
    local_ip_ = getLocalIPAccessibleFromHost(args_.host);
  }

  factory_.reset(new URFactory(args_.host));

  notifier_ = nullptr;

  if (args_.shutdown_on_disconnect)
  {
    LOG_INFO("Notifier: Pipeline disconnect will shutdown the node");
    notifier_ = new ShutdownOnPipelineStoppedNotifier();
  }
  else
  {
    LOG_INFO("Notifier: Pipeline disconnect will be ignored.");
    notifier_ = new IgnorePipelineStoppedNotifier();
  }

  // RT packets
  rt_parser_ = factory_->getRTParser();
  rt_stream_.reset(new URStream(args_.host, UR_RT_PORT));
  rt_prod_.reset(new URProducer<RTPacket>(*rt_stream_, *rt_parser_));
  rt_commander_ = factory_->getCommander(*rt_stream_);
  rt_cons_.reset(new MultiConsumer<RTPacket>(rt_vec_));
  rt_pl_.reset(new Pipeline<RTPacket>(*rt_prod_, *rt_cons_, "RTPacket", *notifier_));

  // Message packets
  state_parser_ = factory_->getStateParser();
  state_stream_.reset(new URStream(args_.host, UR_SECONDARY_PORT));
  state_prod_.reset(new URProducer<StatePacket>(*state_stream_, *state_parser_));
  state_cons_.reset(new MultiConsumer<StatePacket>(state_vec_));
  state_pl_.reset(new Pipeline<StatePacket>(*state_prod_, *state_cons_, "StatePacket", *notifier_));

  LOG_INFO("Starting main loop");

  rt_pl_->run();
  state_pl_->run();

  return true;
}


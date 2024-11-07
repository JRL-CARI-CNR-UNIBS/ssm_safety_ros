/*
Copyright (c) 2020, Marco Faroni
Poitecnico di Milano
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "sensor_msgs/msg/joint_state.hpp"

#include <rdyn_core/primitives.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <std_msgs/msg/string.hpp>

#include "ssm_safety_ros/ssm_ros_base_node_library.h"
#include <velocity_scaling_iso15066/ssm15066.h>


class RobotDescriptionReader
{
protected:
  bool has_one_available_{false}; // set to true when it reads urdf the first time
  bool has_new_available_{true};  // reset every time i need to retrieve a new urdf
  std::string robot_description_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::mutex mtx_;

  void callback(const std_msgs::msg::String& msg);

public:

  bool is_available();

  bool get_robot_description(rclcpp::Node::SharedPtr& node, std::string& robot_description, const double& timeout_secs=10.0, const bool& use_stored_urdf_if_available=true);

};
using RobotDescriptionReaderPtr = std::shared_ptr<RobotDescriptionReader>;


class UnscaledJointTargetNotifier
{
protected:
  bool new_data_available_{false};
  bool first_msg_received_{false};
  size_t n_joints_;
  std::vector<std::string> joint_names_;
  std::vector<double> pos_;
  std::vector<double> vel_;

public:

  UnscaledJointTargetNotifier(const size_t& n_joints, const std::vector<std::string>& joint_names);

  bool is_a_new_data_available();

  bool was_first_msg_received();

  bool get_data(std::vector<double>& pos, std::vector<double>& vel);

  void callback(const sensor_msgs::msg::JointState::SharedPtr msg);

};
using UnscaledJointTargetNotifierPtr = std::shared_ptr<UnscaledJointTargetNotifier>;

class SsmDynamicNode :  public SsmBaseNode
{
protected:

  RobotDescriptionReaderPtr robot_description_reader_;
  UnscaledJointTargetNotifierPtr js_notif_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rdyn::ChainPtr chain_;
  std::vector<std::string> joint_names_;
  size_t nAx_;
  ssm15066::DeterministicSSMPtr ssm_;

public:

  SsmDynamicNode(std::string name);

  virtual bool init() override;

  virtual void spin() override;

};

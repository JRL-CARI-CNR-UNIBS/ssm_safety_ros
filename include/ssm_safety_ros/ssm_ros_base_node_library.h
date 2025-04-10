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

#include "ssm_safety_ros/common.h"

#include <rdyn_core/primitives.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>


class SsmBaseNode :  public rclcpp::Node
{
protected:

  std::string params_ns_;
  std::string js_topic_;

  double sampling_time_{0.02};
  std::string base_frame_;
  std::string tool_frame_;
  std::vector<std::string> test_links_;
  // std::string sphere_radius_;

  // kinematic chain and joint state subscriber
  RobotDescriptionReaderPtr robot_description_reader_;
  rdyn::ChainPtr chain_;
  std::vector<std::string> joint_names_;
  size_t nAx_;

  double time_remove_old_objects_{0.5};
  double last_ovr_{0.0};
  rclcpp::Time last_pose_topic_; // = rclcpp::Time(0);

  std_msgs::msg::Int16 ovr_msg_int_;
  std_msgs::msg::Float32 ovr_msg_float32_;
  std_msgs::msg::Float64 ovr_msg_float64_;

  std_msgs::msg::Float32 dist_msg_float32_;
  std_msgs::msg::Float64 dist_msg_float64_;

  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b_pos_;
  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b_vel_;

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double pos_ovr_change_;
  double neg_ovr_change_;
  //rclcpp::WallRate lp(1.0/st);

  // publisher and subscibers
  JointStateNotifierPtr js_notif_;
  HumanPoseNotifierPtr obstacle_notifier_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ovr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ovr_float_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ovr_float64_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_float64_pub_;

public:

  SsmBaseNode(std::string name);

  virtual bool init();

  virtual void spin()=0;

  virtual void publish_ovr(double& ovr);

  virtual void publish_distance(const double& dist);

  void set_cnr_param_namespace(const std::string& ns);

};

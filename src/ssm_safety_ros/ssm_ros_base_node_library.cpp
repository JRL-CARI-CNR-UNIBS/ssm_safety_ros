/*
Copyright (c) 2025, Marco Faroni
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

#include "ssm_safety_ros/ssm_ros_base_node_library.h"

SsmBaseNode::SsmBaseNode(std::string name): rclcpp::Node(name)
{
  params_ns_ = "/"+name+"/";
  js_topic_ = "/joint_states";
}

bool SsmBaseNode::init()
{

  // get params
  std::string what;

  if (!cnr::param::get(params_ns_+"sampling_time", sampling_time_, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter sampling_time. default = %f. (%s)", sampling_time_, what.c_str());
  }
  if (!cnr::param::get(params_ns_+"base_frame", base_frame_, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter base_frame. default = %s. (%s)", base_frame_.c_str(), what.c_str());
  }
  if (!cnr::param::get(params_ns_+"tool_frame", tool_frame_, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter tool_frame. default = %s. (%s)", tool_frame_.c_str(), what.c_str());
  }
  if (!cnr::param::get(params_ns_+"time_remove_old_objects", time_remove_old_objects_, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter time_remove_old_objects. default = %f. (%s)", time_remove_old_objects_, what.c_str());
  }

  //if (!cnr::param::get(params_ns+"publish_obstacles", publish_obstacles, what))
  //{
  //  RCLCPP_WARN(this->get_logger(), "parameter publish_obstacles undefined. default = %f. (%s)", publish_obstacles);
  //}
  //if (!cnr::param::get(params_ns+"sphere_radius", sphere_radius, what))
  //{
  //  RCLCPP_WARN(this->get_logger(), "parameter sphere_radius undefined. default = %f. (%s)", sphere_radius);
  //}

  pos_ovr_change_=0.25*sampling_time_;
  neg_ovr_change_=2.0*sampling_time_;

  RCLCPP_INFO(this->get_logger(), "creating rosdyn chain");

  // create kinematic chain
  rclcpp::Node::SharedPtr nh = shared_from_this();
  robot_description_reader_ = std::make_shared<RobotDescriptionReader>();
  std::string robot_description;
  if (!robot_description_reader_->get_robot_description(nh,robot_description))
  {
    RCLCPP_FATAL(this->get_logger(), "could not find robot description. FAILED.");
    return false;
  }

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(robot_description);

  if(model == nullptr)
  {
    RCLCPP_FATAL(this->get_logger(), "Cannot load robot_description!");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "urdf model ok");

  chain_ = rdyn::createChain(*model, base_frame_, tool_frame_, grav);
  if (!chain_)
  {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Unable to create a chain between " << base_frame_ << " and " << tool_frame_);
    return false;
  }

  joint_names_ = chain_->getMoveableJointNames();
  nAx_ = joint_names_.size();

  test_links_ = chain_->getLinksName();
  if (!cnr::param::get(params_ns_+"test_links", test_links_, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter dynamic_ssm/test_links. default = ALL. (%s)", what.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "rosdyn chain initialized");

  // init tf buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  ovr_pub_ = this->create_publisher<std_msgs::msg::Int16>("/speed_ovr",1);
  ovr_float_pub_ = this->create_publisher<std_msgs::msg::Float32>("/speed_ovr_float",1);
  ovr_float64_pub_ = this->create_publisher<std_msgs::msg::Float64>("/speed_ovr_float64",1);
  dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("/min_distance_from_poses",1);
  dist_float64_pub_ = this->create_publisher<std_msgs::msg::Float64>("/min_distance_from_poses_float64",1);

  // create subscribers
  obstacle_notifier_ = std::make_shared<HumanPoseNotifier>(base_frame_, tf_buffer_);
  obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/poses", 1, std::bind(&HumanPoseNotifier::callback, obstacle_notifier_, std::placeholders::_1));
  js_notif_ = std::make_shared<JointStateNotifier>(nAx_,joint_names_);
  js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(js_topic_, 1, std::bind(&JointStateNotifier::callback, js_notif_, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ssm_base_node initialized");

  return true;
}

void SsmBaseNode::publish_ovr(double& ovr)
{
  if(ovr<0)
    ovr = 0.0;

  if (ovr>(last_ovr_+pos_ovr_change_))
    ovr=last_ovr_+pos_ovr_change_;
  else if (ovr<(last_ovr_-neg_ovr_change_))
    ovr=last_ovr_-neg_ovr_change_;
  last_ovr_=ovr;

  ovr_msg_int_.data=100*ovr;
  ovr_pub_->publish(ovr_msg_int_);

  ovr_msg_float32_.data=ovr_msg_int_.data;
  ovr_float_pub_->publish(ovr_msg_float32_);

  ovr_msg_float64_.data=ovr_msg_float32_.data;
  ovr_float64_pub_->publish(ovr_msg_float64_);

  RCLCPP_DEBUG(this->get_logger(), "ovr = %f", ovr_msg_float32_.data);

}

void SsmBaseNode::publish_distance(const double& dist)
{
  dist_msg_float32_.data=dist;
  dist_pub_->publish(dist_msg_float32_);

  dist_msg_float64_.data=dist;
  dist_float64_pub_->publish(dist_msg_float64_);
}

void SsmBaseNode::set_cnr_param_namespace(const std::string &ns)
{
  params_ns_ = "/"+ns+"/";
}





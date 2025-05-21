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

#include "ssm_safety_ros/common.h"

SignalHandler::SignalHandler(const Signal& signal, const rclcpp::Node::SharedPtr& node):
  signal_(signal), node_(node)
{
  std::cout << "name: " << signal_.name << std::endl;
  std::cout << "channel: " << signal_.channel << std::endl;
  std::cout << "interface: " << signal_.interface << std::endl;
  std::cout << "st: " << signal_.sampling_time << std::endl;

}

bool SignalHandler::is_alive(){return is_alive_;}

void SignalHandler::init()
{
  std::cout << "init: " << std::endl;
  std::cout << "name: " << signal_.name << std::endl;
  std::cout << "channel: " << signal_.channel << std::endl;
  std::cout << "interface: " << signal_.interface << std::endl;
  std::cout << "st: " << signal_.sampling_time << std::endl;
}

bool SignalHandler::is_active()
{
  return (signal_.normally_closed != is_active_); // logical XOR
}


TopicHandler::TopicHandler(const Signal& signal, const rclcpp::Node::SharedPtr& node):
  SignalHandler(signal, node),
  min_period_(rclcpp::Duration::from_seconds(signal_.sampling_time))
{
  subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
        signal_.channel, 10, std::bind(&TopicHandler::TopicCallback, this, std::placeholders::_1));
  last_call_time_ = node_->now();
  std::cout << "topic handler created!" << std::endl;
}

void TopicHandler::TopicCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_active_ = msg->data;
  is_new_data_available_ = true;
}

bool TopicHandler::is_active()
{
  is_active_ = false;

  rclcpp::Time now = node_->now();
  if ((now - last_call_time_) >= min_period_)
  {
    rclcpp::spin_some(node_->get_node_base_interface());
    if (is_new_data_available_)
    {
      std::cout << "Response: data = " << is_active_ << std::endl;
      is_new_data_available_ = false;
    }
    else
    {
      std::cerr << "Did not receive any new message" << std::endl;
    }
  }
  return this->SignalHandler::is_active();
}

ServiceHandler::ServiceHandler(const Signal& signal, const rclcpp::Node::SharedPtr& node):
  SignalHandler(signal, node),
  min_period_(rclcpp::Duration::from_seconds(signal_.sampling_time))
{
  client_ = node_->create_client<std_srvs::srv::Trigger>(signal_.channel);
  last_call_time_ = node_->now();
  std::cout << "srv handler created!" << std::endl;
}

bool ServiceHandler::is_active()
{
  rclcpp::Time now = node_->now();

  if (!is_alive_)
  {
    if (!client_->wait_for_service(1s))
    {
      std::cerr << "Service not available" << std::endl;
    }
    else
    {
      is_alive_ = true;
    }
  }
  else if ((now - last_call_time_) >= min_period_)
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client_->async_send_request(request);
    last_call_time_ = now;

    if (rclcpp::spin_until_future_complete(this->node_, future, std::chrono::nanoseconds(5*min_period_.nanoseconds())) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = *future.get();
      is_active_ = result.success;
      std::cout << "Response: data = " << result.success << ", msg = " << result.message << std::endl;
    }
    else
    {
      std::cerr << "Service call failed" << std::endl;
      is_alive_ = false;
    }
  }
  return this->SignalHandler::is_active();
}




bool RobotDescriptionReader::is_available()
{
  return has_one_available_;
}

void RobotDescriptionReader::callback(const std_msgs::msg::String& msg)
{
  mtx_.lock();
  robot_description_ = msg.data;
  has_one_available_ = true;
  has_new_available_=true;
  mtx_.unlock();
}

bool RobotDescriptionReader::get_robot_description(rclcpp::Node::SharedPtr& node, std::string& robot_description, const double& timeout_secs, const bool& use_stored_urdf_if_available)
{
  if (this->is_available() && use_stored_urdf_if_available)
  {
    robot_description = robot_description_;
    return true;
  }

  has_new_available_ = false;
  robot_description_sub_ = node->create_subscription<std_msgs::msg::String>("/robot_description", rclcpp::QoS(1).transient_local().reliable(), std::bind(&RobotDescriptionReader::callback, this, std::placeholders::_1));

  auto t0 = rclcpp::Clock{}.now();
  while (!has_new_available_ && (rclcpp::Clock{}.now() - t0).seconds() <= timeout_secs)
  {
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 0.5, "waiting for robot description to come up");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  robot_description_sub_.reset();

  if (has_new_available_)
  {
    robot_description = robot_description_;
    return true;
  }
  else if (this->is_available())
  {
    robot_description = robot_description_;
    RCLCPP_ERROR(node->get_logger(), "could not read up-to-date robot description. returning the last available one.");
  }
  return false;
}


JointStateNotifier::JointStateNotifier(const size_t& n_joints, const std::vector<std::string>& joint_names)
{
  n_joints_ = n_joints;
  joint_names_ = joint_names;
  pos_.resize(n_joints_);
  vel_.resize(n_joints_);
}

bool JointStateNotifier::is_a_new_data_available()
{
  return new_data_available_;
}

bool JointStateNotifier::was_first_msg_received()
{
  return first_msg_received_;
}

bool JointStateNotifier::get_data(std::vector<double>& pos, std::vector<double>& vel)
{
  if (!new_data_available_)
  {
    return false;
  }
  pos = pos_;
  vel = vel_;
  new_data_available_ = false;
  return true;
}

void JointStateNotifier::callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  first_msg_received_=true;

  pos_ = msg->position;
  vel_ = msg->velocity;

  std::vector<std::string> tmp_names = msg->name;
  name_sorting::permutationName(joint_names_,tmp_names,pos_,vel_);
  new_data_available_ = true;
}


HumanPoseNotifier::HumanPoseNotifier(const std::string& base_frame, const tf2_ros::Buffer::SharedPtr& tf_buffer)
{
  base_frame_ = base_frame;
  tf_buffer_ = tf_buffer;
}

bool HumanPoseNotifier::is_a_new_data_available()
{
  return new_data_available_;
}

bool HumanPoseNotifier::was_first_pose_received()
{
  return first_msg_received_;
}

bool HumanPoseNotifier::get_data(Eigen::Matrix<double,3,Eigen::Dynamic>& pc_in_b)
{
  if (!new_data_available_)
  {
    return false;
  }
  pc_in_b = pc_in_b_;
  new_data_available_ = false;
  return true;
}

void HumanPoseNotifier::callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  first_msg_received_=true;

  Eigen::Affine3d T_base_camera;
  T_base_camera.setIdentity();
  geometry_msgs::msg::TransformStamped location_transform;
  tf2::TimePoint t0 = tf2::TimePointZero;


  if (msg->header.frame_id.compare(base_frame_))
  {
    bool success {true};
    for (size_t itrial=0;itrial<50;itrial++)
    {
      try
      {
        location_transform = tf_buffer_->lookupTransform(base_frame_.c_str(), msg->header.frame_id, t0, tf2::Duration(std::chrono::milliseconds(5000)));
      }
      catch (tf2::LookupException ex)
      {
        fprintf(stderr, "[WARNING] Timeout: Unable to find a transform from %s to %s\n", base_frame_.c_str(), msg->header.frame_id.c_str());
        fprintf(stderr, "[WARNING] %s", ex.what());
        success = false;
      }
      catch(std::exception ex)
      {
        fprintf(stderr, "[WARNING] Unable to find a transform from %s to %s\n", base_frame_.c_str(), msg->header.frame_id.c_str());
        fprintf(stderr, "[WARNING] %s", ex.what());
        success = false;
      }
      if (success)
        break;

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if(success)
    {
      T_base_camera = tf2::transformToEigen(location_transform);
    }

  }
  else
  {
    location_transform = tf2::eigenToTransform(T_base_camera);
  }

  pc_in_b_.resize(3,msg->poses.size());
  for (size_t ip=0;ip<msg->poses.size();ip++)
  {
    Eigen::Vector3d point_in_c;
    point_in_c(0)=msg->poses.at(ip).position.x;
    point_in_c(1)=msg->poses.at(ip).position.y;
    point_in_c(2)=msg->poses.at(ip).position.z;
    pc_in_b_.col(ip)=T_base_camera*point_in_c;
  }

#if 0
  tf2_ros::StampedTransform tf_base_camera;

  if (msg->header.frame_id.compare(base_frame_))
  {

    if (not listener.waitForTransform(base_frame_.c_str(),msg->header.frame_id,msg->header.stamp,rclcpp::Duration(0.01)))
    {
      ROS_ERROR_THROTTLE(1,"Poses topic has wrong frame, %s instead of %s. No TF available",poses.header.frame_id.c_str(),base_frame_.c_str());
      error=true;
    }
    else
    {
      listener.lookupTransform(base_frame_,msg->header.frame_id,msg->header.stamp,tf_base_camera);
      tf::poseTFToEigen(tf_base_camera,T_base_camera);
    }
  }
  else
  {
    tf::poseEigenToTF(T_base_camera,tf_base_camera);
  }

  pc_in_b_.resize(3,msg->poses.size());
  for (size_t ip=0;ip<msg->poses.size();ip++)
  {
    Eigen::Vector3d point_in_c;
    point_in_c(0)=msg->poses.at(ip).position.x;
    point_in_c(1)=msg->poses.at(ip).position.y;
    point_in_c(2)=msg->poses.at(ip).position.z;
    pc_in_b_.col(ip)=T_base_camera*point_in_c;
  }
#endif
  new_data_available_ = true;
}




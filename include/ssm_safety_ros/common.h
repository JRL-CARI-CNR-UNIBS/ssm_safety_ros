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

#pragma once

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cnr_param/cnr_param.h>
#include "name_sorting/name_sorting.hpp"

#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;



// define a struct containing the signal's info
struct Signal
{
  std::string name;
  std::string interface;
  std::string channel;
  bool normally_closed;
  double sampling_time;
  std::vector<std::string> areas;
};

// define a struct containing the area's info
struct Area
{
  // TODO: this is ok if we only have two shapes, should use polimorphism
  std::string name;
  double override;
  std::vector<std::vector<double>> corners;
  double radius;
};

// second we specialize a template function from cnr_param to use custom structs
namespace YAML
{
template <>
struct convert<Area>
{
  static Node encode(const Area& rhs)
  {
    Node node;
    node["name"] = rhs.name;
    node["override"] = rhs.override;
    if (rhs.corners.size()>0)
    {
      node["corners"] = rhs.corners;
    }
    else
    {
      node["radius"] = rhs.radius;
    }
    return node;
  }

  static bool decode(const Node& node, Area& rhs)
  {
    if (!node.IsMap() || !node["name"] || !node["override"])
    {
      return false;
    }
    rhs.name = node["name"].as<std::string>();
    rhs.override = node["override"].as<double>();
    if (node["corners"])
    {
      rhs.corners = node["corners"].as<std::vector<std::vector<double>>>();
    }
    else if (node["radius"])
    {
      rhs.radius = node["radius"].as<double>();
    }
    else
    {
      return false;
    }
    return true;
  }
};

template <>
struct convert<Signal>
{
  static Node encode(const Signal& rhs)
  {
    Node node;
    node["name"] = rhs.name;
    node["normally_closed"] = rhs.normally_closed;
    node["interface"] = rhs.interface;
    node["channel"] = rhs.channel;
    node["sampling_time"] = rhs.sampling_time;
    node["areas"] = rhs.areas;
    return node;
  }

  static bool decode(const Node& node, Signal& rhs)
  {
    if (!node.IsMap() || !node["name"] || !node["channel"] || !node["interface"] || !node["areas"])
    {
      std::cerr << "[ERROR] one field missing in signal configuration."
                   " At least the fields 'name', 'channel', 'interface', and 'areas'"
                   " must be present." << std::endl;
      return false;
    }
    rhs.name = node["name"].as<std::string>();
    rhs.interface = node["interface"].as<std::string>();
    rhs.channel = node["channel"].as<std::string>();
    rhs.areas = node["areas"].as<std::vector<std::string>>();

    if (rhs.areas.size()==0)
    {
      std::cerr << "[ERROR] 'areas' array cannot be empty in signal configuration." << std::endl;
      return false;
    }

    if (node["normally_closed"])
    {
      rhs.normally_closed = node["normally_closed"].as<bool>();
    }
    else
    {
      rhs.normally_closed = false;
    }
    if (node["sampling_time"])
    {
      rhs.sampling_time = node["sampling_time"].as<double>();
    }
    else
    {
      rhs.sampling_time = 0.05;
    }


    return true;
  }
};
}  // namespace YAML

// TODO: fare in modo che is_active() ritorni il fatto che il segnale sia attivo o meno
// creare subscriber e service clients e gestire la frequenza di campionamento

class SignalHandler
{
protected:
  bool is_active_{false};
  bool is_alive_{false};

  Signal signal_;
  rclcpp::Node::SharedPtr node_;


public:

  SignalHandler(const Signal& signal, const rclcpp::Node::SharedPtr& node):
    signal_(signal), node_(node)
  {
    std::cout << "name: " << signal_.name << std::endl;
    std::cout << "channel: " << signal_.channel << std::endl;
    std::cout << "interface: " << signal_.interface << std::endl;
    std::cout << "st: " << signal_.sampling_time << std::endl;

  }

  bool is_alive(){return is_alive_;};

  virtual void init()
  {
    std::cout << "init: " << std::endl;
    std::cout << "name: " << signal_.name << std::endl;
    std::cout << "channel: " << signal_.channel << std::endl;
    std::cout << "interface: " << signal_.interface << std::endl;
    std::cout << "st: " << signal_.sampling_time << std::endl;

  };

  virtual bool is_active()
  {
    return (signal_.normally_closed != is_active_); // logical XOR
  }

  //void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

};
using SignalHandlerPtr = std::shared_ptr<SignalHandler>;

class TopicHandler : public SignalHandler
{
public:
  TopicHandler(const Signal& signal, const rclcpp::Node::SharedPtr& node):
    SignalHandler(signal, node)
  {
    std::cout << "topic handler created!" << std::endl;
  }
};

class ServiceHandler : public SignalHandler
{
public:
  ServiceHandler(const Signal& signal, const rclcpp::Node::SharedPtr& node):
    SignalHandler(signal, node),
    min_period_(rclcpp::Duration::from_seconds(signal_.sampling_time))
  {
    client_ = node_->create_client<std_srvs::srv::Trigger>(signal_.channel);
    last_call_time_ = node_->now();
    std::cout << "srv handler created !" << std::endl;
  }

  bool is_active()
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

      if (rclcpp::spin_until_future_complete(this->node_, future) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        auto result = *future.get();
        is_active_ = result.success;
        std::cout << "Response: data = " << result.success << ", msg = " << result.message << std::endl;
      }
      else
      {
        std::cerr << "Service call failed" << std::endl;
      }
    }
    return this->SignalHandler::is_active();
  }

protected:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_call_time_;
  rclcpp::Duration min_period_;

};

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


class JointStateNotifier
{
protected:
  bool new_data_available_{false};
  bool first_msg_received_{false};
  size_t n_joints_;
  std::vector<std::string> joint_names_;
  std::vector<double> pos_;
  std::vector<double> vel_;

public:

  JointStateNotifier(const size_t& n_joints, const std::vector<std::string>& joint_names);

  bool is_a_new_data_available();

  bool was_first_msg_received();

  bool get_data(std::vector<double>& pos, std::vector<double>& vel);

  void callback(const sensor_msgs::msg::JointState::SharedPtr msg);

};
using JointStateNotifierPtr = std::shared_ptr<JointStateNotifier>;

class HumanPoseNotifier
{
protected:
  bool new_data_available_{false};
  bool first_msg_received_{false};
  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b_;
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::string base_frame_;

public:

  HumanPoseNotifier(const std::string& base_frame, const tf2_ros::Buffer::SharedPtr& tf_buffer);

  bool is_a_new_data_available();

  bool was_first_pose_received();

  bool get_data(Eigen::Matrix<double,3,Eigen::Dynamic>& pc_in_b);

  void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  tf2_ros::Buffer::SharedPtr get_tf_buffer(){return tf_buffer_;};

};
using HumanPoseNotifierPtr = std::shared_ptr<HumanPoseNotifier>;

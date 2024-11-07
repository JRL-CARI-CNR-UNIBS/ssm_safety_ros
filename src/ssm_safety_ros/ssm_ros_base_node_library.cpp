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

#include "ssm_safety_ros/ssm_ros_base_node_library.h"

HumanPoseNotifier::HumanPoseNotifier(const std::string& base_frame)
{
  base_frame_ = base_frame;
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




SsmBaseNode::SsmBaseNode(): Node("ssm_node")
{
  // get params
  this->declare_parameter("sampling_time", 0.02);
  sampling_time_ = this->get_parameter("sampling_time").as_double();
  RCLCPP_INFO(this->get_logger(), "sampling time: %f", sampling_time_);

  pos_ovr_change_=0.25*sampling_time_;
  neg_ovr_change_=2.0*sampling_time_;

  this->declare_parameter("base_frame", "base_link");
  base_frame_ = this->get_parameter("base_frame").as_string();
  RCLCPP_INFO_STREAM(this->get_logger(), "base_frame: " << base_frame_);

  this->declare_parameter("tool_frame", "tcp");
  tool_frame_ = this->get_parameter("tool_frame").as_string();
  RCLCPP_INFO_STREAM(this->get_logger(), "tool_frame: " << tool_frame_);

  // this->declare_parameter("publish_obstacles", false);
  // publish_obstacles_ = this->get_parameter("publish_obstacles").as_bool();
  // RCLCPP_INFO(this->get_logger(), "publish_obstacles: %d", publish_obstacles_);

  // this->declare_parameter("sphere_radius", 0.3);
  // sphere_radius_ = this->get_parameter("sphere_radius").as_double();
  // RCLCPP_INFO(this->get_logger(), "sphere_radius: %f", sphere_radius_);

  this->declare_parameter("time_remove_old_objects", 0.5);
  time_remove_old_objects_ = this->get_parameter("time_remove_old_objects").as_double();
  RCLCPP_INFO(this->get_logger(), "time_remove_old_objects: %f", time_remove_old_objects_);

  ovr_pub_ = this->create_publisher<std_msgs::msg::Int16>("/speed_ovr",1);
  ovr_float_pub_ = this->create_publisher<std_msgs::msg::Float32>("/speed_ovr_float",1);
  ovr_float64_pub_ = this->create_publisher<std_msgs::msg::Float64>("/speed_ovr_float64",1);
  dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("/min_distance_from_poses",1);
  dist_float64_pub_ = this->create_publisher<std_msgs::msg::Float64>("/min_distance_from_poses_float64",1);

  obstacle_notifier_ = std::make_shared<HumanPoseNotifier>(base_frame_);
  // CHECK THIS
  obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/poses", 1, std::bind(&HumanPoseNotifier::callback, obstacle_notifier_, std::placeholders::_1));


  RCLCPP_INFO(this->get_logger(), "ssm_base_node initialized");
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



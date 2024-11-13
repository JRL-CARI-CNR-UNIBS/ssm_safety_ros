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

#include "ssm_safety_ros/ssm_ros_fixed_distance_node_library.h"

SsmFixedDistanceNode::SsmFixedDistanceNode(std::string name):
  SsmFixedAreasNode(name)
{
  areas_param_ns_="fixed_distance";
  robot_in_b_pos_xy_.setZero();
}

bool SsmFixedDistanceNode::init()
{
  if (!SsmBaseNode::init())
    return false;

  // create SSM scaling calculator
  ssm_ = std::make_shared<ssm15066::FixedDistanceSSM>();
  ssm_->init();
  ssm_->setPointCloud(pc_in_b_pos_,pc_in_b_vel_);

  loadAreas();
  ssm_->printAreas();

  RCLCPP_INFO(this->get_logger(), "ssm_fixed_areas_node initialized");

  return true;
}

void SsmFixedDistanceNode::spin()
{

  // downcasting to child class to use child method setRobotToolPosition()
  ssm15066::FixedDistanceSSMPtr ssm_child = std::static_pointer_cast<ssm15066::FixedDistanceSSM>(ssm_);

  Eigen::VectorXd q;
  Eigen::VectorXd dq;

  int iter = 0;
  rclcpp::WallRate lp(1.0/sampling_time_);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(this->get_node_base_interface());

    iter++;

    if (obstacle_notifier_->is_a_new_data_available())
    {
      RCLCPP_DEBUG(this->get_logger(),"poses received correctly");

      obstacle_notifier_->get_data(pc_in_b_pos_);
      ssm_child->setPointCloud(pc_in_b_pos_, pc_in_b_vel_);

      last_pose_topic_ = rclcpp::Clock{}.now();

      // compute transformation from base_frame to tool_frame
      geometry_msgs::msg::TransformStamped location_transform;
      tf2::TimePoint t0 = tf2::TimePointZero;

      if (tool_frame_.compare(base_frame_))
      {
        bool success {true};
        for (size_t itrial=0;itrial<50;itrial++)
        {
          try
          {
            auto start = rclcpp::Clock{}.now();
            location_transform = tf_buffer_->lookupTransform(base_frame_.c_str(), tool_frame_.c_str(), t0);
          }
          catch (tf2::LookupException ex)
          {
            fprintf(stderr, "[WARNING] Timeout: Unable to find a transform from %s to %s\n", base_frame_.c_str(), tool_frame_.c_str());
            fprintf(stderr, "[WARNING] %s", ex.what());
            success = false;
          }
          catch(std::exception ex)
          {
            fprintf(stderr, "[WARNING] Unable to find a transform from %s to %s\n", base_frame_.c_str(), tool_frame_.c_str());
            fprintf(stderr, "[WARNING] %s", ex.what());
            success = false;
          }
          if (success)
          {
            RCLCPP_WARN(this->get_logger(),"found tf");
            break;
          }

          rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        if(success)
        {
          robot_in_b_pos_xy_(0) = location_transform.transform.translation.x;
          robot_in_b_pos_xy_(1) = location_transform.transform.translation.y;
          std::cout << "robot pos" << robot_in_b_pos_xy_ << std::endl;
          ssm_child->setRobotToolPosition(robot_in_b_pos_xy_);
        }
      }
    }

    // poses is old
    if ((rclcpp::Clock{}.now()-last_pose_topic_).seconds() > time_remove_old_objects_)
    {
      pc_in_b_pos_.resize(3,0);
      pc_in_b_vel_.resize(3,0);
      ssm_child->setPointCloud(pc_in_b_pos_,pc_in_b_vel_);
    }

    if (!obstacle_notifier_->was_first_pose_received())
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2.0, "poses topic has not been received yet");
    }


    double ovr=ssm_child->computeScaling(q,dq);
    publish_ovr(ovr);
    publish_distance(ssm_child->getDistanceFromClosestPoint());

    lp.sleep();


  }
}

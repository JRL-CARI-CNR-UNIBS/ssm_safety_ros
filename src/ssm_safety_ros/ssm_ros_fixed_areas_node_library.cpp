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

#include "ssm_safety_ros/ssm_ros_fixed_areas_node_library.h"

SsmFixedAreasNode::SsmFixedAreasNode(std::string name):
  SsmBaseNode(name), areas_param_ns_("fixed_areas")
{}

bool SsmFixedAreasNode::init()
{
  if (!SsmBaseNode::init())
    return false;

  // create SSM scaling calculator
  ssm_ = std::make_shared<ssm15066::FixedAreasSSM>();
  ssm_->init();
  ssm_->setPointCloud(pc_in_b_pos_, pc_in_b_vel_);

  loadAreas();
  ssm_->printAreas();


  RCLCPP_INFO(this->get_logger(), "ssm_fixed_areas_node initialized");

  return true;
}

void SsmFixedAreasNode::spin()
{
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
      ssm_->setPointCloud(pc_in_b_pos_, pc_in_b_vel_);

      last_pose_topic_ = rclcpp::Clock{}.now();
    }

    // poses is old
    if ((rclcpp::Clock{}.now()-last_pose_topic_).seconds() > time_remove_old_objects_)
    {
      pc_in_b_pos_.resize(3,0);
      pc_in_b_vel_.resize(3,0);
      ssm_->setPointCloud(pc_in_b_pos_,pc_in_b_vel_);
    }

    if (!obstacle_notifier_->was_first_pose_received())
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2.0, "poses topic has not been received yet");
    }

    double ovr=ssm_->computeScaling(q,dq);
    publish_ovr(ovr);
    // publish_distance(ssm_->getDistanceFromClosestPoint());

    lp.sleep();
  }

}

bool SsmFixedAreasNode::loadAreas()
{
  if (!ssm_)
  {
    RCLCPP_ERROR(this->get_logger(), "ssm_ must be created before using loadAreas()." );
    return false;
  }

  // get areas from yaml
  std::string what;
  std::vector<Area> areas;
  if (!cnr::param::get(params_ns_+areas_param_ns_+"/areas", areas, what))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "could not load parameter fixed_areas/areas." << what);
    return false;
  }

  for (auto& area: areas)
  {
    if (area.override>1.0 || area.override<0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "area %s has inconsistent override value. expected values are between 0.0 and 1.0. received %f. setting 0.0.", area.name.c_str(), area.override);
      area.override=0.0;
    }
    if (area.corners.size()>0)
    {
      ssm_->addArea(area.name, area.corners, area.override);
    }
    else
    {
      ssm_->addArea(area.name, area.radius, area.override);
    }
  }
  return true;
}

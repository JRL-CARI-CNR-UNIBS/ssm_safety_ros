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

#include "ssm_safety_ros/ssm_ros_dynamic_node_library.h"

SsmDynamicNode::SsmDynamicNode(std::string name): SsmBaseNode(name)
{
  js_topic_ = "/unscaled_joint_target";
}

bool SsmDynamicNode::init()
{
  if (!SsmBaseNode::init())
  {
    return false;
  }

  // get params
  std::string what;

  double max_cart_acc = 0.1;
  if (!cnr::param::get(params_ns_+"dynamic_ssm/maximum_cartesian_acceleration", max_cart_acc, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter dynamic_ssm/maximum_cartesian_acceleration. default = %f. (%s)", max_cart_acc, what.c_str());
  }
  double reaction_time = 0.15;
  if (!cnr::param::get(params_ns_+"dynamic_ssm/maximum_cartesian_acceleration", reaction_time, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter dynamic_ssm/reaction_time. default = %f. (%s)", reaction_time, what.c_str());
  }
  double default_human_speed = 0.0;
  if (!cnr::param::get(params_ns_+"dynamic_ssm/default_human_speed", default_human_speed, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter dynamic_ssm/default_human_speed. default = %f. (%s)", default_human_speed, what.c_str());
  }
  double min_protective_dist = 0.3;
  if (!cnr::param::get(params_ns_+"dynamic_ssm/min_protective_dist", min_protective_dist, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter dynamic_ssm/min_protective_dist. default = %f. (%s)", min_protective_dist, what.c_str());
  }
  double min_filtered_dist = 0.1;
  if (!cnr::param::get(params_ns_+"dynamic_ssm/min_filtered_dist", min_filtered_dist, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter dynamic_ssm/min_filtered_dist. default = %f. (%s)", min_filtered_dist, what.c_str());
  }
  bool measure_human_speed = false;
  if (!cnr::param::get(params_ns_+"dynamic_ssm/measure_human_speed", measure_human_speed, what))
  {
    RCLCPP_WARN(this->get_logger(), "could not load parameter dynamic_ssm/measure_human_speed. default = %d. (%s)", measure_human_speed, what.c_str());
  }

  // create SSM scaling calculator
  ssm_ = std::make_shared<ssm15066::DeterministicSSM>(chain_);
  ssm_->setMaxCartesianAcceleration(max_cart_acc);
  ssm_->setReactionTime(reaction_time);
  ssm_->setDefaultHumanSpeed(default_human_speed);
  ssm_->setMinProtectiveDistance(min_protective_dist);
  ssm_->setFilteringSelfDistance(min_filtered_dist);
  ssm_->useMeasuredHumanVelocity(measure_human_speed);

  ssm_->setCheckedRobotLinks(test_links_);
  ssm_->init();
  ssm_->setPointCloud(pc_in_b_pos_,pc_in_b_vel_);

  RCLCPP_INFO(this->get_logger(), "ssm_dynamic_node initialized");

  return true;
}

void SsmDynamicNode::spin()
{
  Eigen::VectorXd q(nAx_);
  Eigen::VectorXd dq(nAx_);
  q.setZero();
  dq.setZero();
  std::vector<double> pos(nAx_);
  std::vector<double> vel(nAx_);

  // int iter = 0;
  rclcpp::WallRate lp(1.0/sampling_time_);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(this->get_node_base_interface());
    double ovr=0;
    if (js_notif_->is_a_new_data_available())
    {
      js_notif_->get_data(pos, vel);
      for (unsigned int iax=0;iax<nAx_;iax++)
      {
        q(iax)=pos.at(iax);
        dq(iax)=vel.at(iax);
      }
    }

    #if 0
    /* Print links and poses for debug */
    if (iter==500 || iter==0)
    {
      std::vector<std::string> links = chain_->getLinksName();
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> Tbl = chain_->getTransformations(q);

      std::vector<std::string> poi_names = ssm_->getPoiNames();

      RCLCPP_INFO(this->get_logger(), "Links: %lu", links.size());
      for (unsigned int idx=0;idx<links.size();idx++)
      {
        std::cout << idx << ": " << links.at(idx) << std::endl;
      }

      RCLCPP_INFO(this->get_logger(), "Links used for safety check: %lu", poi_names.size());
      for (unsigned int idx=0;idx<poi_names.size();idx++)
      {
        std::cout << idx << ": " << poi_names.at(idx)  << std::endl;
      }

      for (unsigned int idx=0;idx<links.size();idx++)
      {
        //consider only links inside the poi_names_ list
        if(std::find(poi_names.begin(),poi_names.end(),links[idx])>=poi_names.end())
          continue;

        double x = Tbl.at(idx).translation()(0);
        double y = Tbl.at(idx).translation()(1);
        double z = Tbl.at(idx).translation()(2);
        std::cout << "#" << idx << " : " << links.at(idx) << "\t";
        std::cout << "[x,y,z] = " << "[" << x << ", " << y << ", " << z << "]" << std::endl;
      }

      RCLCPP_INFO(this->get_logger(), "Min distance from poses: %f", ssm_->getDistanceFromClosestPoint());

      RCLCPP_INFO(this->get_logger(), "Number of axis: %zu. Joints:", joint_names_.size());
      for (unsigned int idx=0;idx<joint_names_.size();idx++)
      {
        std::cout << idx << ": " << joint_names_.at(idx) << " : " << q(idx) << std::endl;
      }

      iter=1;
    }
    iter++;
    #endif

    if (obstacle_notifier_->is_a_new_data_available())
    {
      RCLCPP_DEBUG(this->get_logger(),"poses received correctly");

      obstacle_notifier_->get_data(pc_in_b_pos_);
      ssm_->setPointCloud(pc_in_b_pos_, pc_in_b_vel_);

      last_pose_topic_ = rclcpp::Clock{}.now();

      // TODO
      #if 0
      if(publish_obstacles)
      {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[0] = sphere_radius;

        moveit_msgs::CollisionObject collision_object;

        collision_object.header.frame_id=base_frame;
        collision_object.header.stamp=ros::Time::now();
        //        collision_object.pose.orientation.w=1;
        //        collision_object.id="skeleton_obs";
        if (poses.poses.size()>0)
        {
          pose_frame_id = "skeleton_obj_";
          pose_frame_id.append(poses.header.frame_id);

          collision_object.id = pose_frame_id;
          collision_object.operation = collision_object.ADD;

          for (size_t ip=0;ip<poses.poses.size();ip++)
          {
            geometry_msgs::Pose p;
            p.position.x=pc_in_b.col(ip)(0);
            p.position.y=pc_in_b.col(ip)(1);
            p.position.z=pc_in_b.col(ip)(2);
            p.orientation.w=1.0;
            collision_object.primitive_poses.push_back(p);
            collision_object.primitives.push_back(primitive);
          }
        }
        else
        {
          collision_object.operation = collision_object.REMOVE;
        }

        std::vector<moveit_msgs::CollisionObject> collision_objects; //addCollisionObect requires a vector
        collision_objects.push_back(collision_object);
        planning_scene_interface.addCollisionObjects(collision_objects);
      }
      #endif
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
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "poses topic has not been received yet");
    }

    if (!js_notif_->was_first_msg_received())
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "topic js_topic_ has not been received yet");
    }
    else
    {
      ovr=ssm_->computeScaling(q,dq);
    }

    publish_ovr(ovr);
    publish_distance(ssm_->getDistanceFromClosestPoint());

    lp.sleep();


  }
}

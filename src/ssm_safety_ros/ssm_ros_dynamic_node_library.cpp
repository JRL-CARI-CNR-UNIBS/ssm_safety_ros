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
#include "name_sorting/name_sorting.hpp"

RobotDescriptionReader::RobotDescriptionReader(const rclcpp::Node::SharedPtr& node)
{
  node_=node;
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

bool RobotDescriptionReader::get_robot_description(std::string& robot_description, const double& timeout_secs, const bool& use_stored_urdf_if_available)
{
  if (this->is_available() && use_stored_urdf_if_available)
  {
    robot_description = robot_description_;
    return true;
  }

  has_new_available_ = false;
  robot_description_sub_ = node_->create_subscription<std_msgs::msg::String>("/robot_description", rclcpp::QoS(1).transient_local().reliable(), std::bind(&RobotDescriptionReader::callback, this, std::placeholders::_1));

  auto t0 = rclcpp::Clock{}.now();
  while (!has_new_available_ && (rclcpp::Clock{}.now() - t0).seconds() <= timeout_secs)
  {
    rclcpp::spin_some(node_->get_node_base_interface());
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 0.5, "waiting for robot description to come up");
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
    RCLCPP_ERROR(node_->get_logger(), "could not read up-to-date robot description. returning the last available one.");
  }
  return false;
}


UnscaledJointTargetNotifier::UnscaledJointTargetNotifier(const size_t& n_joints, const std::vector<std::string>& joint_names)
{
  n_joints_ = n_joints;
  joint_names_ = joint_names;
  pos_.resize(n_joints_);
  vel_.resize(n_joints_);
}

bool UnscaledJointTargetNotifier::is_a_new_data_available()
{
  return new_data_available_;
}

bool UnscaledJointTargetNotifier::was_first_msg_received()
{
  return first_msg_received_;
}

bool UnscaledJointTargetNotifier::get_data(std::vector<double>& pos, std::vector<double>& vel)
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

void UnscaledJointTargetNotifier::callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  first_msg_received_=true;

  pos_ = msg->position;
  vel_ = msg->velocity;

  std::vector<std::string> tmp_names = msg->name;
  name_sorting::permutationName(joint_names_,tmp_names,pos_,vel_);
  new_data_available_ = true;
}

SsmDynamicNode::SsmDynamicNode(): SsmBaseNode()
{
  // create kinematic chain
  robot_description_reader_ = std::make_shared<RobotDescriptionReader>(shared_from_this());
  std::string robot_description;
  if (!robot_description_reader_->get_robot_description(robot_description))
  {
    RCLCPP_FATAL(this->get_logger(), "could not find robot description. FAILED.");
  }

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(robot_description);

  if(model == nullptr)
  {
    RCLCPP_FATAL(this->get_logger(), "Cannot load robot_description!");
  }
  RCLCPP_INFO(this->get_logger(), "urdf model ok");

  rdyn::ChainPtr chain = rdyn::createChain(*model, base_frame_, tool_frame_, grav);
  if (!chain)
  {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Unable to create a chain between " << base_frame_ << " and " << tool_frame_);
  }

  RCLCPP_INFO(this->get_logger(), "rosdyn chain ok");

  // get params
  std::vector<std::string> test_links = chain_->getLinksName();
  this->declare_parameter("test_links", test_links);
  this->get_parameter("test_links", test_links);

  this->declare_parameter("maximum_cartesian_acceleration", 0.1);
  double max_cart_acc = this->get_parameter("maximum_cartesian_acceleration").as_double();
  RCLCPP_INFO(this->get_logger(), "maximum_cartesian_acceleration: %f", max_cart_acc);

  this->declare_parameter("reaction_time", 0.15);
  double reaction_time = this->get_parameter("reaction_time").as_double();
  RCLCPP_INFO(this->get_logger(), "reaction_time: %f", reaction_time);

  this->declare_parameter("default_human_speed", 0.0);
  double default_human_speed = this->get_parameter("default_human_speed").as_double();
  RCLCPP_INFO(this->get_logger(), "default_human_speed: %f", default_human_speed);

  this->declare_parameter("min_protective_dist", 0.3);
  double min_protective_dist = this->get_parameter("min_protective_dist").as_double();
  RCLCPP_INFO(this->get_logger(), "min_protective_dist: %f", min_protective_dist);

  this->declare_parameter("min_filtered_dist", 0.15);
  double min_filtered_dist = this->get_parameter("min_filtered_dist").as_double();
  RCLCPP_INFO(this->get_logger(), "min_filtered_dist: %f", min_filtered_dist);

  this->declare_parameter("measure_human_speed", false);
  bool measure_human_speed = this->get_parameter("measure_human_speed").as_bool();
  RCLCPP_INFO(this->get_logger(), "measure_human_speed: %d", measure_human_speed);

  // create SSM scaling calculator
  ssm_ = std::make_shared<ssm15066::DeterministicSSM>(chain_);
  ssm_->setMaxCartesianAcceleration(max_cart_acc);
  ssm_->setReactionTime(reaction_time);
  ssm_->setDefaultHumanSpeed(default_human_speed);
  ssm_->setMinProtectiveDistance(min_protective_dist);
  ssm_->setFilteringSelfDistance(min_filtered_dist);
  ssm_->useMeasuredHumanVelocity(measure_human_speed);
  ssm_->setCheckedRobotLinks(test_links);
  ssm_->init();
  ssm_->setPointCloud(pc_in_b_pos_,pc_in_b_vel_);

  joint_names_ = chain_->getMoveableJointNames();
  nAx_ = joint_names_.size();

  // create subscribers
  js_notif_ = std::make_shared<UnscaledJointTargetNotifier>(nAx_,joint_names_);
  js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/unscaled_joint_target", 1, std::bind(&UnscaledJointTargetNotifier::callback, js_notif_, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ssm_dynamic_node initialized");
}

void SsmDynamicNode::spin()
{
  Eigen::VectorXd q(nAx_);
  Eigen::VectorXd dq(nAx_);
  q.setZero();
  dq.setZero();
  std::vector<double> pos(nAx_);
  std::vector<double> vel(nAx_);

  int iter = 0;
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

    if (obstacle_notifier_->is_a_new_data_available())
    {
      RCLCPP_DEBUG(this->get_logger(),"poses received correctly");

      obstacle_notifier_->get_data(pc_in_b_pos_);
      ssm_->setPointCloud(pc_in_b_pos_, pc_in_b_vel_);

      last_pose_topic_ = rclcpp::Time(0);

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
    if ((rclcpp::Time(0)-last_pose_topic_).seconds() > time_remove_old_objects_)
    {
      pc_in_b_pos_.resize(3,0);
      pc_in_b_vel_.resize(3,0);
      ssm_->setPointCloud(pc_in_b_pos_,pc_in_b_vel_);
    }

    if (!obstacle_notifier_->was_first_pose_received())
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2.0, "poses topic has not been received yet");
    }

    if (!js_notif_->was_first_msg_received())
    {
      RCLCPP_INFO(this->get_logger(),"unscaled joint target topic has not been received yet");
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

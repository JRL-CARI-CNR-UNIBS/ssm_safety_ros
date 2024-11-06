#include <cstdio>
#include <rclcpp/rclcpp.hpp>

//#include <rosdyn_core/primitives.h>
#include <rdyn_core/primitives.h>
#include <rdyn_core/urdf_parser.h>

//#include <velocity_scaling_iso15066/ssm15066.h>
#include <random>
//#include <subscription_notifier/subscription_notifier.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>

#include <tf2_ros/transform_listener.h>
#include "name_sorting/name_sorting.hpp"
#include <Eigen/Dense>
#include "cnr_param/cnr_param.h"
#include "cnr_logger/cnr_logger.h"

using std::placeholders::_1;


//#include <tf_conversions/tf_eigen.h>

//#include <moveit/planning_scene_interface/planning_scene_interface.h>

class SsmIso15066 : public rclcpp::Node
{
public:
  SsmIso15066() : Node("ssm_iso15066")
  {

    // publishers and subscribers
    ovr_pb_ = this->create_publisher<std_msgs::msg::Int64>("/safe_ovr_1",1);
    ovr_float_pb_ = this->create_publisher<std_msgs::msg::Float32>("/safe_ovr_1_float",1);
    ovr_float64_pb_ = this->create_publisher<std_msgs::msg::Float64>("/safe_ovr_1_float64",1);
    dist_pb_ = this->create_publisher<std_msgs::msg::Float32>("/min_distance_from_poses",1);
    dist_float64_pb_ = this->create_publisher<std_msgs::msg::Float64>("/min_distance_from_poses_float64",1);

    obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/poses", 1, std::bind(&SsmIso15066::poses_notifier, this, _1));
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/unscaled_joint_target", 1, std::bind(&SsmIso15066::unscaled_joint_target_notifier, this, _1));

  }



protected:
  std::shared_ptr<cnr_logger::TraceLogger> logger_;

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr ovr_pb_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ovr_float_pb_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ovr_float64_pb_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pb_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_float64_pb_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

  geometry_msgs::msg::PoseArray::SharedPtr poses_;
  sensor_msgs::msg::JointState::SharedPtr unscaled_js_;

  std::string base_frame_;
  std::string tool_frame_;
  rdyn::ChainPtr chain_;
  double pos_ovr_change_;
  double neg_ovr_change_;

  bool publish_obstacles_{false};
  double sphere_radius_{0.3};
  double time_remove_old_objects_{0.5};
  double loop_rate_;

  bool unscaled_joint_target_received_{false};
  bool poses_received_{false};

  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  size_t nAx_;
  std::vector<std::string> joint_names_;
  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b_;

  rclcpp::Time last_pose_topic_;




  void poses_notifier(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    Eigen::Affine3d T_base_camera;
    T_base_camera.setIdentity();
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
    ssm.setPointCloud(pc_in_b_);
    poses_received_ = true;
    last_pose_topic_ = rclcpp::Time(0);
  }

  void unscaled_joint_target_notifier(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::vector<double> pos = msg->position;
    std::vector<double> vel = msg->velocity;

    std::vector<std::string> tmp_names = msg->name;
    name_sorting::permutationName(joint_names_,tmp_names,pos,vel);

    for (unsigned int iax=0;iax<nAx_;iax++)
    {
      q_(iax) = pos.at(iax);
      dq_(iax) = vel.at(iax);
    }
    unscaled_joint_target_received_ = true;
  }

  bool config()
  {
    // Loading all params
    std::string what, param_name;

    double st=0.02;
    param_name = "sampling_time";
    if(not cnr::param::get(param_name, st, what, true,
    {cnr::param::ModulesID::MAPPED_FILE,cnr::param::ModulesID::ROS1,cnr::param::ModulesID::ROS2}))
    {
      CNR_WARN(logger_, "Cannot load " << param_name + " parameter.\n" << what);
    }

    param_name = "base_frame";
    if(not cnr::param::get(param_name, base_frame_, what, true,
    {cnr::param::ModulesID::MAPPED_FILE,cnr::param::ModulesID::ROS1,cnr::param::ModulesID::ROS2}))
    {
      CNR_ERROR(logger_, "Cannot load " << param_name + " parameter.\n" << what);
      throw std::invalid_argument("Cannot load " + param_name + " parameter.");
      return false;
    }

    param_name = "tool_frame";
    if(not cnr::param::get(param_name, tool_frame_, what, true,
    {cnr::param::ModulesID::MAPPED_FILE,cnr::param::ModulesID::ROS1,cnr::param::ModulesID::ROS2}))
    {
      CNR_ERROR(logger_, "Cannot load " << param_name + " parameter.\n" << what);
      throw std::invalid_argument("Cannot load " + param_name + " parameter.");
      return false;
    }

    // Loading publishing obstacles information
    param_name = "publish_obstacles";
    if(not cnr::param::get(param_name, publish_obstacles_, what, true,
    {cnr::param::ModulesID::MAPPED_FILE,cnr::param::ModulesID::ROS1,cnr::param::ModulesID::ROS2}))
    {
      CNR_WARN(logger_, param_name + " parameter undefined. Default = " << publish_obstacles_ << "\n" << what);
    }

    param_name = "sphere_radius";
    if(publish_obstacles_)
    {
      if(not cnr::param::get(param_name, sphere_radius_, what, true,
      {cnr::param::ModulesID::MAPPED_FILE,cnr::param::ModulesID::ROS1,cnr::param::ModulesID::ROS2}))
      {
        CNR_WARN(logger_, param_name + " parameter undefined. Default = " << sphere_radius_ << "\n" << what);
      }
    }

    param_name = "time_remove_old_objects";
    if(not cnr::param::get(param_name, time_remove_old_objects_, what, true,
    {cnr::param::ModulesID::MAPPED_FILE,cnr::param::ModulesID::ROS1,cnr::param::ModulesID::ROS2}))
    {
      CNR_WARN(logger_, param_name + " parameter undefined. Default = " << time_remove_old_objects_ << "\n" << what);
    }

    // TODO
    //tf::TransformListener listener;
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // create chain
    urdf::ModelInterface model;
    // TODO
    //model =  *urdf::parseURDFFile(urdf_path.string());
    Eigen::Vector3d grav;
    grav << 0, 0, -9.806;
    chain_ = rdyn::createChain(model,base_frame_,tool_frame_,grav);
    if (!chain_)
    {
      CNR_ERROR(logger_, "Unable to create a chain between " << base_frame_ << " and " << tool_frame_ << ".\n" << what);
      return false;
    }

    joint_names_=chain_->getMoveableJointNames();
    nAx_=joint_names_.size();

    Eigen::VectorXd velocity_limits(nAx_);
    Eigen::VectorXd inv_velocity_limits(nAx_);
    Eigen::VectorXd upper_limits(nAx_);
    Eigen::VectorXd lower_limits(nAx_);
    for (size_t iAx=0; iAx<nAx_; iAx++)
    {
      upper_limits(iAx)    = model.getJoint(joint_names_.at(iAx))->limits->upper;
      lower_limits(iAx)    = model.getJoint(joint_names_.at(iAx))->limits->lower;
      velocity_limits(iAx) = model.getJoint(joint_names_.at(iAx))->limits->velocity;
      inv_velocity_limits(iAx) = 1./model.getJoint(joint_names_.at(iAx))->limits->velocity;
    }

    // Loading links to test for ssm
    std::vector<std::string> test_links;
    param_name = "test_links";
    if(not cnr::param::get(param_name, test_links, what, true,
    {cnr::param::ModulesID::MAPPED_FILE,cnr::param::ModulesID::ROS1,cnr::param::ModulesID::ROS2}))
    {
      test_links = chain_->getLinksName();
      CNR_WARN(logger_, param_name + " undefined. Use all available links.\n" << what);
    }

    // Initialize parameters
    pos_ovr_change_ = 0.25*st;
    neg_ovr_change_ = 2.0*st;
    loop_rate_ = 1.0/st;
    //rclcpp::WallRate lp(1.0/st);

    return true;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SsmIso15066>());
  rclcpp::shutdown();
  return 0;
}




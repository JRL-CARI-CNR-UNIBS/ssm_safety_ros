#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include <rdyn_core/primitives.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <velocity_scaling_iso15066/ssm15066.h>
#include <random>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "name_sorting/name_sorting.hpp"
#include <Eigen/Dense>
#include "cnr_param/cnr_param.h"
#include "cnr_logger/cnr_logger.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

//#include <moveit/planning_scene_interface/planning_scene_interface.h>

/*TODO:
- planning scene interface
*/

class RobotDescriptionNotifier
{
protected:
  bool new_data_available_{false};
  std::string robot_description_;

public:

  bool is_a_new_data_available()
  {
    return new_data_available_;
  }

  bool get_data(std::string& robot_description_string)
  {
    if (!new_data_available_)
    {
      return false;
    }
    robot_description_string = robot_description_;
    new_data_available_ = false;
    return true;
  }

  void callback(const std_msgs::msg::String& msg)
  {
    robot_description_ = msg.data;
    new_data_available_ = true;
  }

};

class ObstacleNotifier
{
protected:
  bool new_data_available_{false};
  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b_;
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::string base_frame_;

  // TODO
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

public:

  ObstacleNotifier(const std::string& base_frame)
  {
    base_frame_ = base_frame;
  }

  bool is_a_new_data_available()
  {
    return new_data_available_;
  }

  bool get_data(Eigen::Matrix<double,3,Eigen::Dynamic>& pc_in_b)
  {
    if (!new_data_available_)
    {
      return false;
    }
    pc_in_b = pc_in_b_;
    new_data_available_ = false;
    return true;
  }

  void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {

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


};

class JointTargetNotifier
{
protected:
  bool new_data_available_{false};
  std::vector<double> pos_;
  std::vector<double> vel_;
  size_t nAx_;
  std::vector<std::string> joint_names_;

public:
  JointTargetNotifier(const size_t& nAx, const std::vector<std::string>& joint_names)
  {
    nAx_ = nAx;
    joint_names_ = joint_names;
    pos_.resize(nAx_);
    vel_.resize(nAx_);
  }

  bool is_a_new_data_available()
  {
    return new_data_available_;
  }

  bool get_data(std::vector<double>& pos, std::vector<double>& vel)
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

  void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    pos_ = msg->position;
    vel_ = msg->velocity;

    std::vector<std::string> tmp_names = msg->name;
    name_sorting::permutationName(joint_names_,tmp_names,pos_,vel_);
    new_data_available_ = true;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("velocity_scaling_iso_15066_node");
  //std::shared_ptr<cnr_logger::TraceLogger> logger;
  std::string what, param_name;

  RCLCPP_ERROR(node->get_logger(), "init node");

  node->declare_parameter("sampling_time", 0.02);
  double st = node->get_parameter("sampling_time").as_double();
  RCLCPP_INFO(node->get_logger(), "sampling time: %f", st);

  double pos_ovr_change=0.25*st;
  double neg_ovr_change=2.0*st;
  rclcpp::WallRate lp(1.0/st);

  node->declare_parameter("base_frame", "base_link");
  std::string base_frame = node->get_parameter("base_frame").as_string();
  RCLCPP_INFO_STREAM(node->get_logger(), "base_frame: " << base_frame);

  node->declare_parameter("tool_frame", "tcp");
  std::string tool_frame = node->get_parameter("tool_frame").as_string();
  RCLCPP_INFO_STREAM(node->get_logger(), "tool_frame: " << tool_frame);

  // Loading urdf and rdyn chain
  RCLCPP_INFO(node->get_logger(), "Recovering robot_description from topic");

  RobotDescriptionNotifier robot_description_notif;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub = node->create_subscription<std_msgs::msg::String>("/robot_description", rclcpp::QoS(1).transient_local().reliable(), std::bind(&RobotDescriptionNotifier::callback, &robot_description_notif, _1));

  while (!robot_description_notif.is_a_new_data_available())
  {
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_WARN(node->get_logger(), "waiting for robot description to come up");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  std::string robot_description;
  robot_description_notif.get_data(robot_description);

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(robot_description);

  if(model == nullptr)
  {
    RCLCPP_ERROR(node->get_logger(), "Cannot load robot_description!");
    return 0;
  }
  RCLCPP_INFO(node->get_logger(), "urdf model ok");

  rdyn::ChainPtr chain = rdyn::createChain(*model, base_frame, tool_frame, grav);
  if (!chain)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Unable to create a chain between " << base_frame << " and " << tool_frame << ".\n" << what);
    return 0;
  }

  RCLCPP_INFO(node->get_logger(), "rosdyn chain ok");

  // Loading publishing obstacles information
  node->declare_parameter("publish_obstacles", false);
  bool publish_obstacles = node->get_parameter("publish_obstacles").as_bool();
  RCLCPP_INFO(node->get_logger(), "publish_obstacles: %d", publish_obstacles);

  node->declare_parameter("sphere_radius", 0.3);
  double sphere_radius = node->get_parameter("sphere_radius").as_double();
  RCLCPP_INFO(node->get_logger(), "sphere_radius: %f", sphere_radius);

  node->declare_parameter("time_remove_old_objects", 0.5);
  double time_remove_old_objects = node->get_parameter("time_remove_old_objects").as_double();
  RCLCPP_INFO(node->get_logger(), "time_remove_old_objects: %f", time_remove_old_objects);

  std::vector<std::string> joint_names = chain->getMoveableJointNames();
  size_t nAx=joint_names.size();

  Eigen::VectorXd velocity_limits(nAx);
  Eigen::VectorXd inv_velocity_limits(nAx);
  Eigen::VectorXd upper_limits(nAx);
  Eigen::VectorXd lower_limits(nAx);
  for (size_t iAx=0; iAx<nAx; iAx++)
  {
    upper_limits(iAx)    = model->getJoint(joint_names.at(iAx))->limits->upper;
    lower_limits(iAx)    = model->getJoint(joint_names.at(iAx))->limits->lower;
    velocity_limits(iAx) = model->getJoint(joint_names.at(iAx))->limits->velocity;
    inv_velocity_limits(iAx) = 1./model->getJoint(joint_names.at(iAx))->limits->velocity;
  }

  // Loading links to test for ssm

  std::vector<std::string> test_links = chain->getLinksName();
  node->declare_parameter("test_links", test_links);
  node->get_parameter("test_links", test_links);

  node->declare_parameter("maximum_cartesian_acceleration", 0.1);
  double max_cart_acc = node->get_parameter("maximum_cartesian_acceleration").as_double();
  RCLCPP_INFO(node->get_logger(), "maximum_cartesian_acceleration: %f", max_cart_acc);

  node->declare_parameter("reaction_time", 0.15);
  double reaction_time = node->get_parameter("reaction_time").as_double();
  RCLCPP_INFO(node->get_logger(), "reaction_time: %f", reaction_time);

  node->declare_parameter("default_human_speed", 0.0);
  double default_human_speed = node->get_parameter("default_human_speed").as_double();
  RCLCPP_INFO(node->get_logger(), "default_human_speed: %f", default_human_speed);

  node->declare_parameter("min_protective_dist", 0.3);
  double min_protective_dist = node->get_parameter("min_protective_dist").as_double();
  RCLCPP_INFO(node->get_logger(), "min_protective_dist: %f", min_protective_dist);

  node->declare_parameter("min_filtered_dist", 0.15);
  double min_filtered_dist = node->get_parameter("min_filtered_dist").as_double();
  RCLCPP_INFO(node->get_logger(), "min_filtered_dist: %f", min_filtered_dist);

  node->declare_parameter("measure_human_speed", false);
  bool measure_human_speed = node->get_parameter("measure_human_speed").as_bool();
  RCLCPP_INFO(node->get_logger(), "measure_human_speed: %d", measure_human_speed);

  // publishers and subscribers

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ovr_pb = node->create_publisher<std_msgs::msg::Int16>("/speed_ovr",1);
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ovr_float_pb = node->create_publisher<std_msgs::msg::Float32>("/speed_ovr_float",1);
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ovr_float64_pb = node->create_publisher<std_msgs::msg::Float64>("/speed_ovr_float64",1);
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pb = node->create_publisher<std_msgs::msg::Float32>("/min_distance_from_poses",1);
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_float64_pb = node->create_publisher<std_msgs::msg::Float64>("/min_distance_from_poses_float64",1);

  ObstacleNotifier obstacle_notif(base_frame);
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub = node->create_subscription<geometry_msgs::msg::PoseArray>("/poses", 1, std::bind(&ObstacleNotifier::callback, &obstacle_notif, _1));
  JointTargetNotifier js_notif(nAx,joint_names);
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub = node->create_subscription<sensor_msgs::msg::JointState>("/unscaled_joint_target", 1, std::bind(&JointTargetNotifier::callback, &js_notif, _1));

  bool unscaled_joint_target_received=false;
  bool poses_received=false;

  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b_pos;
  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b_vel;

  ssm15066::DeterministicSSM ssm(chain);
  ssm.setMaxCartesianAcceleration(max_cart_acc);
  ssm.setReactionTime(reaction_time);
  ssm.setDefaultHumanSpeed(default_human_speed);
  ssm.setMinProtectiveDistance(min_protective_dist);
  ssm.setFilteringSelfDistance(min_filtered_dist);
  ssm.useMeasuredHumanVelocity(measure_human_speed);
  ssm.setCheckedRobotLinks(test_links);
  ssm.init();

  ssm.setPointCloud(pc_in_b_pos,pc_in_b_vel);

  Eigen::VectorXd q(nAx);
  Eigen::VectorXd dq(nAx);
  q.setZero();
  dq.setZero();

  std::vector<double> pos(nAx);
  std::vector<double> vel(nAx);

  std_msgs::msg::Int16 ovr_msg;
  ovr_msg.data = 0;
  double last_ovr = 0;

  std::string pose_frame_id;

  int iter = 0;
  rclcpp::Time last_pose_topic = rclcpp::Time(0);


  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    double ovr=0;
    bool error=false;
    if (js_notif.is_a_new_data_available())
    {
      js_notif.get_data(pos, vel);
      for (unsigned int iax=0;iax<nAx;iax++)
      {
        q(iax)=pos.at(iax);
        dq(iax)=vel.at(iax);
      }
      unscaled_joint_target_received=true;
    }

    /* Print links and poses for debug */
    if (iter==500 || iter==0)
    {
      std::vector<std::string> links = chain->getLinksName();
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> Tbl = chain->getTransformations(q);

      std::vector<std::string> poi_names = ssm.getPoiNames();

      RCLCPP_INFO(node->get_logger(), "Links: %lu", links.size());
      for (unsigned int idx=0;idx<links.size();idx++)
      {
        std::cout << idx << ": " << links.at(idx) << std::endl;
      }

      RCLCPP_INFO(node->get_logger(), "Links used for safety check: %lu", poi_names.size());
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

      RCLCPP_INFO(node->get_logger(), "Min distance from poses: %f", ssm.getDistanceFromClosestPoint());

      RCLCPP_INFO(node->get_logger(), "Number of axis: %zu. Joints:", joint_names.size());
      for (unsigned int idx=0;idx<joint_names.size();idx++)
      {
        std::cout << idx << ": " << joint_names.at(idx) << " : " << q(idx) << std::endl;
      }

      iter=1;
    }
    iter++;

    if (obstacle_notif.is_a_new_data_available())
    {
      RCLCPP_DEBUG(node->get_logger(),"poses received correctly");

      obstacle_notif.get_data(pc_in_b_pos);
      ssm.setPointCloud(pc_in_b_pos, pc_in_b_vel);

      poses_received=true;
      last_pose_topic = rclcpp::Time(0);

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
    if ((rclcpp::Time(0)-last_pose_topic).seconds() > time_remove_old_objects)
    {
      pc_in_b_pos.resize(3,0);
      pc_in_b_vel.resize(3,0);
      ssm.setPointCloud(pc_in_b_pos,pc_in_b_vel);
    }

    if (not unscaled_joint_target_received)
    {
      RCLCPP_INFO(node->get_logger(),"unscaled joint target topic has not been received yet");
    }
    if (not poses_received)
    {
      RCLCPP_INFO(node->get_logger(),"poses topic has not been received yet");
    }

    if (unscaled_joint_target_received)
    {
      ovr = 1.0;
      ovr=ssm.computeScaling(q,dq);
    }
    if (error)
      ovr=0.0;

    if(ovr<0)
      ovr = 0.0;

    if (ovr>(last_ovr+pos_ovr_change))
      ovr=last_ovr+pos_ovr_change;
    else if (ovr<(last_ovr-neg_ovr_change))
      ovr=last_ovr-neg_ovr_change;
    last_ovr=ovr;

    ovr_msg.data=100*ovr;
    ovr_pb->publish(ovr_msg);

    std_msgs::msg::Float32 msg_float;
    msg_float.data=ovr_msg.data;
    ovr_float_pb->publish(msg_float);
    RCLCPP_DEBUG(node->get_logger(), "ovr = %hd", ovr_msg.data);

    msg_float.data=ssm.getDistanceFromClosestPoint();
    dist_pb->publish(msg_float);

    std_msgs::msg::Float64 msg_float64;
    msg_float64.data=ovr_msg.data;
    ovr_float64_pb->publish(msg_float64);

    msg_float64.data=ssm.getDistanceFromClosestPoint();
    dist_float64_pb->publish(msg_float64);

    lp.sleep();
  }

  return 0;
}




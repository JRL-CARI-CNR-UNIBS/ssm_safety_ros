#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "ssm_safety_ros/ssm_ros_base_node_library.h"
#include "ssm_safety_ros/ssm_ros_dynamic_node_library.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  SsmDynamicNode node2;
  //node.spin();
  SsmBaseNode node;

  rclcpp::shutdown();
  return 0;
  
}




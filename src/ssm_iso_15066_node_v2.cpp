#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "ssm_safety_ros/ssm_ros_base_node_library.h"
#include "ssm_safety_ros/ssm_ros_dynamic_node_library.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  //node.spin();
  // SsmBaseNode node;

  std::cout << "first node created" << std::endl;

  std::shared_ptr<SsmDynamicNode> node2 = std::make_shared<SsmDynamicNode>("ssm_node");
  node2->init();
  node2->spin();


  rclcpp::shutdown();
  return 0;
  
}




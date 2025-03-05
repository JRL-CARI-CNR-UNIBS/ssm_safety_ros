#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "ssm_safety_ros/ssm_ros_base_node_library.h"
#include "ssm_safety_ros/ssm_ros_dynamic_node_library.h"
#include "ssm_safety_ros/ssm_ros_fixed_areas_node_library.h"
#include "ssm_safety_ros/ssm_ros_fixed_distance_node_library.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string what;

  std::string configuration;
  if (!cnr::param::get("/safety_configuration/name", configuration, what))
  {
    std::cerr << "could not load safety configuration name. " << what << std::endl;
    return -1;
  }

  std::string type;
  if (!cnr::param::get("/safety_configuration/type", type, what))
  {
    std::cerr << "could not load safety configuration type. " << what << std::endl;
    return -1;
  }

  std::shared_ptr<SsmBaseNode> node;
  if (!type.compare("dynamic_ssm"))
  {
    node = std::make_shared<SsmDynamicNode>(configuration);
  }
  else if (!type.compare("fixed_areas"))
  {
    node = std::make_shared<SsmFixedAreasNode>(configuration);
  }
  else if (!type.compare("fixed_distance"))
  {
    node = std::make_shared<SsmFixedDistanceNode>(configuration);
  }
  else
  {
    std::cerr << "unknown safety configuration type. received " << configuration
              << ". available options are [dynamic_ssm, fixed_areas, fixed_distance]"
              << std::endl;
    return -1;
  }

  node->init();
  node->spin();


  rclcpp::shutdown();
  return 0;
  
}




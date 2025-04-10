#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "ssm_safety_ros/ssm_ros_signals_node_library.h"

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

  bool safety_signals;
  if (!cnr::param::get("/safety_configuration/safety_signals", safety_signals, what))
  {
    std::cerr << "could not load safety_signals param. " << what << std::endl;
    return -1;
  }
  
  if (!safety_signals)
  {
    std::cerr << "safety_signals is set to false. exiting the safety signals node." << what << std::endl;
    return -1;
  }
  
  std::string signals_configuration;
  if (!cnr::param::get("/safety_configuration/signals_configuration", signals_configuration, what))
  {
    std::cerr << "could not load signals_configuration name. " << what << std::endl;
    return -1;
  }
  
  std::shared_ptr<SsmSignalsNode> node;
  node = std::make_shared<SsmSignalsNode>(configuration, signals_configuration);
  
  node->init();
  node->spin();

  rclcpp::shutdown();
  return 0;
  
}




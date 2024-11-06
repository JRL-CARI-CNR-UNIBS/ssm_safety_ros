cmake_minimum_required(VERSION 3.8)
project(ssm_safety_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-funroll-loops -Wall -Ofast)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(name_sorting REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rdyn_core REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS Core)
find_package(cnr_param REQUIRED)
find_package(cnr_logger REQUIRED)
find_package(velocity_scaling_iso15066 REQUIRED)


#tf_conversions
#moveit_ros_planning_interface

include_directories(
   #include
   ${EIGEN3_INCLUDE_DIRS}
   ${rdyn_core_INCLUDE_DIRS}
 )



add_executable(velocity_scaling_iso_15066_node src/velocity_scaling_iso_15066_node.cpp)

ament_target_dependencies(velocity_scaling_iso_15066_node PUBLIC
    rclcpp
    tf2_ros
    std_msgs
    geometry_msgs 
    sensor_msgs 
    name_sorting 
    #trajectory_msgs 
    tf2_eigen
)

target_link_libraries(velocity_scaling_iso_15066_node PUBLIC 
    #rclcpp
    ${tf2_ros_LIBRARIES}
    #tf2_ros
    ${std_msgs_LIBRARIES}
    #std_msgs 
    #geometry_msgs 
    ${geometry_msgs_LIBRARIES}
    #sensor_msgs 
    #name_sorting 
    ${name_sorting_LIBRARIES}
    #trajectory_msgs 
    Eigen3::Eigen
    rdyn_core::rdyn_core
    cnr_param::cnr_param
    cnr_logger::cnr_logger
    velocity_scaling_iso15066::velocity_scaling_iso15066
)

target_compile_features(velocity_scaling_iso_15066_node PUBLIC c_std_99 cxx_std_17)  
target_include_directories(velocity_scaling_iso_15066_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS velocity_scaling_iso_15066_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()

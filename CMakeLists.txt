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
find_package(trajectory_msgs REQUIRED)
find_package(name_sorting REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rdyn_core REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS Core)
find_package(velocity_scaling_iso15066 REQUIRED)
find_package(cnr_param REQUIRED)
#moveit_ros_planning_interface

# CREATE PKG LIBRARY
add_library(${PROJECT_NAME} SHARED
    src/${PROJECT_NAME}/ssm_ros_base_node_library.cpp
    src/${PROJECT_NAME}/ssm_ros_dynamic_node_library.cpp
    src/${PROJECT_NAME}/ssm_ros_fixed_areas_node_library.cpp
    src/${PROJECT_NAME}/ssm_ros_fixed_distance_node_library.cpp
    )
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    tf2_ros
    tf2_eigen
    std_msgs
    geometry_msgs 
    sensor_msgs
    trajectory_msgs
    name_sorting
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    )
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    rclcpp_components
    tf2_ros
    tf2_eigen
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    name_sorting
    std_msgs
    rdyn_core
    Eigen3
    velocity_scaling_iso15066
    cnr_param
    )
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    rdyn_core::rdyn_core
    velocity_scaling_iso15066::velocity_scaling_iso15066
    cnr_param::cnr_param
    )

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)


# NODE

add_executable(ssm_safety_node
    src/ssm_safety_node.cpp
    #src/${PROJECT_NAME}/ssm_ros_dynamic_node_library.cpp
    #src/${PROJECT_NAME}/ssm_ros_base_node_library.cpp
    )
ament_target_dependencies(ssm_safety_node
    rclcpp
    )
target_link_libraries(ssm_safety_node
    ${PROJECT_NAME}
    )
target_include_directories(ssm_safety_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(ssm_safety_node PUBLIC c_std_99 cxx_std_17)
install(TARGETS ssm_safety_node
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

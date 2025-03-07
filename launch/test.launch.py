
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,ExecuteProcess 
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
import xacro

def generate_launch_description():

    declared_arguments = []

    ld = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('ssm_safety_ros'),
        'config',
        'example_params.yaml'
    )

    ssm_node = Node(
        package="ssm_safety_ros",
        name="ssm_safety_node",
        executable="ssm_safety_node",
        #parameters=[config],
        output="screen"
    )

    #'cnr_param_server -p $(find-pkg-share graph_ros_tests)/config/test_solver.yaml'

    load_params = ExecuteProcess(cmd=['cnr_param_server', '-p', str(config)])
    
    ld.add_action(ssm_node)
    ld.add_action(load_params)

    return ld




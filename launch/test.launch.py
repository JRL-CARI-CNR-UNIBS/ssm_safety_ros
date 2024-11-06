
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
    
    ssm_params = PathJoinSubstitution(
        [
            FindPackageShare("ssm_safety_ros"),
            "config",
            "example_params.yaml",
        ]
    )

    config = os.path.join(
        get_package_share_directory('ssm_safety_ros'),
        'config',
        'example_params.yaml'
    )
    
    ssm_node = Node(
        package="ssm_safety_ros",
        name="velocity_scaling_iso_15066_node",
        executable="velocity_scaling_iso_15066_node",
        parameters=[config],
        output="screen"
    )
    
    nodes_to_start = [
        ssm_node,
    ]

    return LaunchDescription( nodes_to_start )




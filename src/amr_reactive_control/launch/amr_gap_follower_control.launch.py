# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    
    package_name = 'amr_reactive_control'

    gap_follower_node = Node(package=package_name, 
                    executable='gap_follower_control',
    )

    

    return LaunchDescription([
        gap_follower_control
    ])
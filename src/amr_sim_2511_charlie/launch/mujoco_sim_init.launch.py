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

    package_name = 'amr_sim_2511_charlie'
    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','mux.yaml')



    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params],
    )

    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )


    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params],
                    # parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/cmd_vel_mux_output')],  # Remap the output topic
    )

    mujoco_simulation = Node(package='amr_sim_2511_charlie', 
                    executable='mujoco_car_simulation_init',
                    output='screen'
    )


    return LaunchDescription([
        twist_mux_node,
        teleop_node,
        joy_node,
        mujoco_simulation
    ])
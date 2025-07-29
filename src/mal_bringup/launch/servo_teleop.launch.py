from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os
import datetime

"""
Launch simulation with one arm.

The package can be launched with 'ros2 launch ats_bringup gz_sim_one_arm.launch.py'
"""
logging = False
major_frequency = 25.

def generate_launch_description():
    ld = LaunchDescription()
    
    #Servo driver
    param_file = os.path.join(get_package_share_directory('mal_bringup'), 'config', 'feetech_ros2.yaml')
    servo_driver = Node(
        package="feetech_ros2",
        executable="feetech_ros2_interface",
        name="feetech_ros2_interface",
        output="screen",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "fatal"] # Suppress most logs
    )
    ld.add_action(servo_driver)

    # Keyboard teleop
    keyboard_teleop = Node(
        package='uam_teleop',
        executable='man_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[
            {'servo_increment_deg': 3.0},
        ],
        arguments=["--ros-args", "--log-level", "info"]
        )
    ld.add_action(keyboard_teleop)

    # ROSBAG logging
    rosbag_record = []
    if logging:
        rosbag_name = 'ros2bag_sim_'+datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        rosbag_path = f'/home/martijn/aerial_tactile_servoing/data/rosbags/{rosbag_name}'
        rosbag_record.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', rosbag_path, '-a'], 
            output='screen', 
            log_cmd=True,
        ))

    return ld

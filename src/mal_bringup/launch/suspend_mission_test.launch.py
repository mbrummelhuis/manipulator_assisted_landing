from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory
import os
import datetime

"""
Launch simulation with one arm.

The package can be launched with 'ros2 launch ats_bringup gz_sim_one_arm.launch.py'
"""
logging = True
major_frequency = 25.

def generate_launch_description():
    ld = LaunchDescription()
    
    #Servo driver
    param_file = os.path.join(get_package_share_directory('mal_bringup'), 'config', 'feetech_ros2_two_arms.yaml')
    servo_driver = Node(
        package="feetech_ros2",
        executable="feetech_ros2_interface",
        name="feetech_ros2_interface",
        output="screen",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "fatal"] # Suppress most logs
    )
    ld.add_action(servo_driver)

    # Manipulator kinematic controller
    manipulator_controller = Node(
        package='manipulator_controller',
        executable='manipulator_controller',
        name="manipulator_controller",
        output='screen',
        parameters=[
            {'minimum_pivot_distance': 1.2}, # 1.0 is about the lowest you should go
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )
    ld.add_action(manipulator_controller)

    # Mission director
    mission_director = Node(
        package='mission_director',
        executable='mission_director_suspend',
        name='mission_director_suspend',
        output='screen',
        parameters=[
            {'frequency': major_frequency},
            {'probing_speed': 0.05},
            {'probing_direction': [0., 0., -1.]}
        ],
        arguments=["--ros-args", "--log-level", "info"] # Log level info

    )
    ld.add_action(mission_director)

    # ROSBAG logging
    if logging:
        rosbag_name = 'ros2bag_suspend_'+datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        rosbag_path = f'/ros2_ws/data/rosbags/{rosbag_name}'
        rosbag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', rosbag_path, '-a'],
            output='screen',
            log_cmd=True,
        )
        ld.add_action(rosbag_process)

    return ld

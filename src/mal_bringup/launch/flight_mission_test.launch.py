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

    # Mission director
    mission_director = Node(
        package='mission_director',
        executable='mission_director_flight',
        name='mission_director_flight',
        output='screen',
        parameters=[
            {'frequency': major_frequency},
            {'position_clip': 2.5},
            {'takeoff_altitude': -1.5},
            {'probing_speed': 0.05},
            {'probing_direction': [0., 0., 1.]}
        ],
        arguments=["--ros-args", "--log-level", "info"] # Log level info

    )
    ld.add_action(mission_director)

    # ROSBAG logging
    rosbag_record = []
    if logging:
        rosbag_name = 'ros2bag_mal_'+datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        rosbag_path = f'/ros2/manipulator_assisted_landing/data/rosbags/{rosbag_name}'
        rosbag_record.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', rosbag_path, '-a'], 
            output='screen', 
            log_cmd=True,
        ))

    return ld

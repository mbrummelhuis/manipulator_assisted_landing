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
    # Add the paths to the simulation launch file
    sim_launch_path = os.path.join(get_package_share_directory('px4_uam_sim'), 'launch', 'gz_martijn.launch.py')
    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch_path))
    ld.add_action(gz_sim)
    
    #Controller nodes
    # mal_teleop = Node(
    #     package='mal_teleop',
    #     executable='mal_teleop',
    #     name='mal_teleop',
    #     output='screen',
    #     parameters=[
    #         {'drone_increment_m': 0.1},
    #         {'servo_increment_deg': 10.},
    #         {'position_clip': 3.5}
    #     ],
    #     arguments=['--ros-args', '--log-level', 'info']
    # )
    # ld.add_action(mal_teleop)

    # contact_detection = Node(
    #     package='mal_contact_detecion',
    #     executable='mal_contact_detection',
    #     name='cd_sim_mal',
    #     output='screen',
    #     parameters=[
    #         {'frequency': 25.},
    #         {'difference_threshold': 1.0},
    #         {'acc_threshold': 10.0}
    #     ],
    #     arguments=['--ros-args', '--log-level', 'info']
    # )
    # ld.add_action(contact_detection)

    sim_remapper = Node(
        package='px4_uam_sim',
        executable='sim_remapper.py',
        name='sim_remapper',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(sim_remapper)

    
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

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
probing_direction_body = [0., 0., 1.]

def generate_launch_description():
    ld = LaunchDescription()
    # Add the paths to the simulation launch file
    sim_launch_path = os.path.join(get_package_share_directory('px4_uam_sim'), 'launch', 'gz_martijn.launch.py')
    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch_path))
    ld.add_action(gz_sim)
    
    # Controller nodes
    mission_director = Node(
        package='mission_director',
        executable='mission_director_flight_mission',
        name='mission_director_flight_mission',
        output='screen',
        parameters=[
            {'frequency': major_frequency},
            {'position_clip': 3.0},
            {'takeoff_altitude': -0.7},
            {'probing_direction': probing_direction_body},
            {'probing_speed': 0.01}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(mission_director)

    # Simple wrench observer/contact detector and localizer
    contact_detector = Node(
        package='contact_detection_localization',
        executable='wrench_observer_simple',
        name='wrench_observer_simple',
        output='screen',
        parameters=[
            {'frequency': 100.0},
            {'gain_force': 1.0}, # Should be unity following the dynamics
            {'alpha_force': 0.4}, # 1 is no filtering
            {'gain_torque': 1.0}, # Should be unity following the dynamics
            {'alpha_torque': 0.4}, # 1 is no filtering
            {'alpha_angular_velocity': 0.3},
            {'alpha_accelerometer': 0.25},
            {'force_contact_threshold': 4.5}, # [N] net linear force necessary to conclude contact
            {'torque_contact_threshold': 0.4}, # [Nm] net momentnecessary to conclude contact
            {'alpha_motor_inputs': 0.3}, # 1 is no filtering
            {'angle_threshold': 45.},
            {'probing_direction': probing_direction_body},
            {'contact_timeout_sec': 0.5},
        ],
        arguments=["--ros-args", "--log-level", "info"] # Log level info
    )
    ld.add_action(contact_detector)

    # Landing planner
    landing_planner = Node(
        package='landing_planner',
        executable='landing_planner',
        name='landing_planner',
        output='screen',
        parameters=[
            {'dimension': 2},
            {'landing_offset': 1.0} # Vertical landing start point above surface centroid
        ],
        arguments=["--ros-args", "--log-level", "info"] # Log level info
    )
    ld.add_action(landing_planner)

    # Manipulator kinematic controller
    manipulator_controller = Node(
        package='manipulator_controller',
        executable='manipulator_controller',
        name="MCon",
        output='screen',
        arguments=["--ros-args", "--log-level", "info"]
    )
    ld.add_action(manipulator_controller)

    sim_remapper = Node(
        package='px4_uam_sim',
        executable='sim_remapper.py',
        name='sim_remapper',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(sim_remapper)

    
    # ROSBAG logging
    if logging:
        rosbag_name = 'ros2bag_mal_'+datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        rosbag_path = f'/ros2_ws/data/rosbags/{rosbag_name}'
        rosbag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', rosbag_path, '-a'],
            output='screen',
            log_cmd=True,
        )
        ld.add_action(rosbag_process)

    return ld

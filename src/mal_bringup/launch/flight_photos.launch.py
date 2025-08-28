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
major_frequency = 50.
md_name = 'mission_director_photos'
probing_direction_body = [0., 0., 1.]

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
        name="MCon",
        output='screen',
        parameters=[
            {'minimum_pivot_distance': 1.}
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )
    ld.add_action(manipulator_controller)

    # Wrench estimator
    contact_detector = Node(
        package='contact_detection_localization',
        executable='wrench_observer_simple',
        name='wrench_observer_simple',
        output='screen',
        parameters=[
            {'frequency': 100.0},
            {'gain_force': 1.0}, # Should be unity following the dynamics
            {'alpha_force': 0.15}, # 1 is no filtering
            {'gain_torque': 1.0}, # Should be unity following the dynamics
            {'alpha_torque': 0.15}, # 1 is no filtering
            {'alpha_angular_velocity': 0.2},
            {'alpha_accelerometer': 0.2},
            {'force_contact_threshold': 10.0}, # [N] net linear force necessary to conclude contact, simple has no force threshold contact detection
            {'torque_contact_threshold': 0.65}, # [Nm] net moment necessary to conclude contact
            {'alpha_motor_inputs': 0.2}, # 1 is no filtering
            {'angle_threshold': 45.},
            {'probing_direction': probing_direction_body},
            {'contact_timeout_sec': 0.65}, # For debouncing
        ],
        arguments=["--ros-args", "--log-level", "error"] # Log level info
    )
    ld.add_action(contact_detector)

    # Mission director
    mission_director = Node(
        package='mission_director',
        executable=md_name,
        name='mission_director',
        output='screen',
        parameters=[
            {'frequency': major_frequency},
            {'position_clip': 3.0},
            {'takeoff_altitude': -1.4},
            {'probing_speed': 0.05}, # 
            {'probing_direction': probing_direction_body}
        ],
        arguments=["--ros-args", "--log-level", "info"] # Log level info

    )
    ld.add_action(mission_director)

    # ROSBAG logging
    if logging:
        rosbag_name = 'ros2bag_reference_'+datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        rosbag_path = f'/ros2_ws/data/rosbags/{rosbag_name}'
        rosbag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', rosbag_path, '-a'],
            output='screen',
            log_cmd=True,
        )
        ld.add_action(rosbag_process)

    return ld

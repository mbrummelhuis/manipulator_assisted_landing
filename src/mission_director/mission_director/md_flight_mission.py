import rclpy
from rclpy.node import Node
import datetime

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from numpy import pi, clip
import numpy as np

from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand

from feetech_ros2.srv import SetMode, SetMaxSpeed

L_1 = 0.110
L_2 = 0.330
L_3 = 0.273

class MissionDirectorPy(Node):
    def __init__(self):
        super().__init__('md_flight')

        # Parameters
        self.declare_parameter('frequency', 25.0)
        self.declare_parameter('position_clip', 0.0)
        self.declare_parameter('takeoff_altitude', -1.5)
        self.declare_parameter('probing_speed', 0.05)
        self.declare_parameter('probing_direction', [0., 0., 1.])
        self.declare_parameter('landing_speed', 0.1)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # PX4 publishers
        self.publisher_vehicle_trajectory_setpoint = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.publisher_offboard_control_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',10)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

       # PX4 subscribers
        self.subscriber_vehicle_status = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.subscriber_vehicle_odometry = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.subscriber_vehicle_local_position = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        
        # Servo interfacing
        self.publisher_servo_state = self.create_publisher(JointState, '/servo/in/state', 10)
        self.subscriber_joint_states = self.create_subscription(JointState, '/servo/out/state', self.joint_states_callback, 10)
        self.servo_mode_client = self.create_client(SetMode, 'set_servo_mode')
        while not self.servo_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for servo set mode service')
        self.mode_set_req = SetMode.Request()
        self.servo_max_speed_client = self.create_client(SetMaxSpeed, '/set_servo_max_speed')
        while not self.servo_max_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for servo set max speed service')
        self.max_speed_set_req = SetMaxSpeed.Request()

        # Manipulator interface
        self.publisher_manipulator_positions = self.create_publisher(TwistStamped, '/manipulator/in/position', 10)
        self.publisher_manipulator_velocities = self.create_publisher(TwistStamped, '/manipulator/in/velocity', 10)

        # Landing planner output
        self.subscriber_contact_point = self.create_subscription(Int32, '/contact/out/contact_point', self.contact_point_callback, 10)
        self.subscriber_landing_point = self.create_subscription(TrajectorySetpoint, '/landing/out/start_location', self.landing_point_callback, qos_profile)
        self.subscriber_landing_manipulator = self.create_subscription(TwistStamped, '/landing/out/manipulator', self.landing_manipulator_callback, 10)

        # Mission director in/output
        self.subscriber_input_state = self.create_subscription(Int32, '/md/input', self.input_state_callback, 10)
        self.publisher_md_state = self.create_publisher(Int32, '/md/state', 10)

        # Set initial data
        self.FSM_state = 'entrypoint'
        self.dry_test = False
        self.first_state_loop = True
        self.input_state = 0
        self.position_clip = self.get_parameter('position_clip').get_parameter_value().double_value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        self.armed = False
        self.offboard = False
        self.killed = False
        self.contact_counter = 0
        self.x_setpoint = 0.0
        self.y_setpoint = 0.0
        self.heading_setpoint = 0.0
        self.x_setpoint_contact = 0.0
        self.y_setpoint_contact = 0.0
        self.contact_altitude = -0.8
        self.contact_y_offset = -1.5
        self.retract_right = 1.
        self.retract_left = 1.
        self.xyz_setpoint1 = None
        self.xyz_setpoint2 = None
        self.last_contact_time_right = datetime.datetime.now()
        self.last_contact_time_left = datetime.datetime.now()
        self.most_recent_contact_point = None
        self.landing_speed = self.get_parameter('landing_speed').get_parameter_value().double_value
        self.arm_1_positions = np.array([0.0, 0.0, 0.0])
        self.arm_1_velocities = np.array([0.0, 0.0, 0.0])
        self.arm_1_effort = np.array([0.0, 0.0, 0.0])
        self.arm_2_positions = np.array([0.0, 0.0, 0.0])
        self.arm_2_velocities = np.array([0.0, 0.0, 0.0])
        self.arm_2_effort = np.array([0.0, 0.0, 0.0])

        self.arm_1_nominal = np.array([0.0, 0.35, 0.1]) # Nominal XYZ posiiton in FRD body frame. Make Y larger than L1 + L2, for downwards
        #self.arm_1_nominal = np.array([0.0, 0.53, -0.35]) # For upwards probing
        self.arm_2_nominal = np.array([0.0, -0.35, 0.1]) # Nominal XYZ posiiton in FRD body frame. Make Y larger than L1 + L2
       # self.arm_2_nominal = np.array([0.0, -0.53, -0.35])
        self.previous_ee_1 = self.arm_1_nominal
        self.previous_ee_2 = self.arm_2_nominal
        
        self.manipulator1_landing_position = None
        self.manipulator2_landing_position = None
        self.landing_start_position = None

        self.probing_direction_body = np.array(self.get_parameter('probing_direction').get_parameter_value().double_array_value)
        self.probing_speed = self.get_parameter('probing_speed').get_parameter_value().double_value
        self.get_logger().info(f'probing downward speed {self.probing_speed}')
        self.get_logger().info(f'Probing body velocity vector: {self.probing_direction_body}')
        self.workspace_radius = L_1+L_2+L_3 + 0.1
        self.get_logger().info(f'Maximum workspace radius {self.workspace_radius}')

        self.vehicle_local_position = VehicleLocalPosition()
        self.state_start_time = datetime.datetime.now()
        self.counter = 0

        # Timer -- always last
        self.counter = 0
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        match self.FSM_state:

            case('entrypoint'): # Entry point - wait for position fix
                self.publishMDState(0)
                if self.first_state_loop:
                    self.get_logger().info("Waiting for position fix")
                    self.first_state_loop = False

                self.x_setpoint = self.vehicle_local_position.x
                self.y_setpoint = self.vehicle_local_position.y
                self.heading_setpoint = self.vehicle_local_position.heading
                self.publishOffboardPositionMode()

                # State transition
                if (self.x_setpoint != 0.0 and self.y_setpoint != 0.0) or self.input_state == 1:
                    self.get_logger().info(f'Got position fix! \t x: {self.x_setpoint:.3f} [m] \t y: {self.y_setpoint:.3f} [m] \t {np.rad2deg(self.heading_setpoint):.2f} [deg]')
                    self.transition_state(new_state='wait_for_servo_driver')

            case('wait_for_servo_driver'):
                self.publishMDState(1)
                # State transition
                if (datetime.datetime.now() - self.state_start_time).seconds > 2 or self.input_state == 1:
                    self.transition_state('default_config')

            case('default_config'):
                self.move_arms_to_joint_position(
                    pi/2, 0.0, -1.82,
                    -pi/2, 0.0, 1.82)
                self.publishMDState(2)
                if self.first_state_loop:
                    self.srv_set_servo_max_speed(0.5)
                    self.first_state_loop = False

                if (datetime.datetime.now() - self.state_start_time).seconds > 3 or self.input_state == 1:
                    self.transition_state('wait_for_arm_offboard')

            case('wait_for_arm_offboard'):
                self.publishMDState(3)
                self.publishOffboardPositionMode()
                self.move_arms_to_joint_position(
                    pi/3, 0.0, 1.6,
                    -pi/3, 0.0, -1.6)

                # State transition
                if self.armed and not self.offboard:
                    self.get_logger().info('Armed but not offboard -- waiting')
                elif not self.armed and self.offboard:
                    self.get_logger().info('Not armed but offboard -- waiting')
                elif (self.armed and self.offboard) or self.input_state == 1:
                    self.transition_state('takeoff')

            case('takeoff'): # Takeoff - wait for takeoff altitude
                self.publishMDState(4)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.heading_setpoint)
                self.move_arms_to_joint_position(
                    pi/3, 0.0, 1.6,
                    -pi/3, 0.0, -1.6)
                current_altitude = self.vehicle_local_position.z

                # First state loop
                if self.first_state_loop:
                    self.get_logger().info(f'Vehicle local position heading: {self.vehicle_local_position.heading}')
                    self.get_logger().info(f'Takeoff altitude: {self.takeoff_altitude}')
                    self.first_state_loop = False

                # State transition
                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif abs(current_altitude)+0.1 > abs(self.takeoff_altitude) or self.input_state==1:
                    self.transition_state('move_arms_to_start_position')

            case('move_arms_to_start_position'):
                self.publishMDState(5)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.heading_setpoint)

                # First time the state is called, find the probing direction and align the arms
                if self.first_state_loop:
                    # Get the angle w.r.t. [0., 0., 1.] to align the arms
                    downward_direction = np.array([0., 1.])
                    probing_direction_yz = np.array([self.probing_direction_body[1], self.probing_direction_body[2]])
                    self.get_logger().info(f'Downward direction: {downward_direction}, length {len(downward_direction)}, shape {downward_direction.shape}, type {type(downward_direction)}')
                    self.get_logger().info(f'Probing direction YZ: {probing_direction_yz}, length {len(probing_direction_yz)}, shape {probing_direction_yz.shape}, type {type(probing_direction_yz)}')
                    
                    probing_direction_angle = self.signed_angle_2d(downward_direction, probing_direction_yz)

                    self.get_logger().info(f"Probing direction angle w.r.t. positive body z about body x: {np.rad2deg(probing_direction_angle):.3f} [deg]")
                    rotated_arm1_nominal = self.Rx(probing_direction_angle, self.arm_1_nominal)
                    rotated_arm2_nominal = self.Rx(probing_direction_angle, self.arm_2_nominal)
                    self.get_logger().info(f"Arm 1 rotated setpoint (XYZ): {rotated_arm1_nominal[0]:.2f}, {rotated_arm1_nominal[1]:.2f}, {rotated_arm1_nominal[2]:.2f}")
                    self.get_logger().info(f"Arm 2 rotated setpoint (XYZ): {rotated_arm2_nominal[0]:.2f}, {rotated_arm2_nominal[1]:.2f}, {rotated_arm2_nominal[2]:.2f}")
                    self.move_arms_to_xyz_position(rotated_arm1_nominal, rotated_arm2_nominal)
                    self.first_state_loop = False
                    # TODO figure out way to enforce elbow up condition here

                # State transition
                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif (datetime.datetime.now() - self.state_start_time).seconds > 4 or self.input_state == 1:
                    self.srv_set_servo_max_speed(0.2)
                    self.transition_state('move_to_probing_location')  
                    self.xyz_setpoint1 = self.position_forward_kinematics(*self.arm_1_positions)
                    self.xyz_setpoint2 = self.position_forward_kinematics(*self.arm_2_positions)
                    self.x_setpoint_contact = self.x_setpoint
                    self.y_setpoint_contact = self.contact_y_offset
                    self.retract_left = 1.
                    self.retract_right = 1. # Reset because maybe earlier false readings cause them to be negative

            case('move_to_probing_location'):
                self.publishMDState(6)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint_contact, self.y_setpoint_contact, self.contact_altitude, self.heading_setpoint)

                # First time the state is called, find the probing direction and align the arms
                if self.first_state_loop:
                    self.get_logger().info(f"Moving to probing location")
                    self.first_state_loop = False

                # State transition
                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif (datetime.datetime.now() - self.state_start_time).seconds > 5 or self.input_state == 1:
                    self.transition_state('probing')  
                    self.xyz_setpoint1 = self.position_forward_kinematics(*self.arm_1_positions)
                    self.xyz_setpoint2 = self.position_forward_kinematics(*self.arm_2_positions)

            # Verify after here
            case('probing'):
                self.publishMDState(11)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint_contact, self.y_setpoint_contact, self.contact_altitude, self.heading_setpoint)

                if self.first_state_loop:
                    self.contact_counter = 0
                    self.first_state_loop = False
                    self.landing_start_position = None # Reset landing start position

                # Retract if indicated, otherwise probe in forward direction, stop arm if nearing workspace edge
                self.move_arms_to_xyz_position(self.xyz_setpoint1, self.xyz_setpoint2)
                velocity_vector_body = self.probing_direction_body*self.probing_speed

                # Arm 1: If in workspace, calculate new setpoint
                if np.linalg.norm(self.xyz_setpoint1) < self.workspace_radius and self.retract_right > 0.:
                    self.xyz_setpoint1 += velocity_vector_body*self.timer_period
                elif np.linalg.norm(self.xyz_setpoint1) > self.workspace_radius:
                    self.get_logger().info(f"Arm 1 out of workspace radius", throttle_duration_sec=1)
                else:
                    self.xyz_setpoint1 = self.xyz_setpoint1 # keep setpoint the same

                # Arm 2: If in workspace, calculate new setpoint
                if np.linalg.norm(self.xyz_setpoint2) < self.workspace_radius and self.retract_left > 0.: # If everything all right
                    self.xyz_setpoint2 += velocity_vector_body*self.timer_period
                elif np.linalg.norm(self.xyz_setpoint2) > self.workspace_radius: # If out of workspace and retracting
                    self.get_logger().info(f"Arm 2 out of workspace radius", throttle_duration_sec=1) # if out of workspace but not retracting
                else:
                    self.xyz_setpoint2 = self.xyz_setpoint2 # keep setpoint the same

                # State transitions
                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif self.landing_start_position is not None:
                    self.get_logger().info(f"[PROBING] LANDING POSITION: {self.landing_start_position.position[0]:.2F} {self.landing_start_position.position[1]:.2F} {self.landing_start_position.position[2]:.2F} \n Publish 2 to continue", throttle_duration_sec=1)
                    self.srv_set_servo_max_speed(0.5)
                    self.transition_state('wait_for_landing_cmd')
                elif self.retract_right < -1.5 or self.retract_left < -1.5:
                    self.transition_state("stabilize_after_contact")
                elif (np.linalg.norm(self.xyz_setpoint1) > self.workspace_radius or np.linalg.norm(self.xyz_setpoint2) > self.workspace_radius) and self.landing_start_position is None:
                    self.contact_altitude += 0.1 # If out of workspace and no landing location, try again hovering 10 cm lower.
                    self.transition_state("move_arms_to_start_position")
            
            case("stabilize_after_contact"):
                self.publishMDState(12)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint_contact, self.y_setpoint_contact, self.contact_altitude-0.4, self.heading_setpoint)
                arm_1_fk = self.position_forward_kinematics(*self.arm_1_positions) # pass current arm position as reference to 'freeze in place'
                arm_2_fk = self.position_forward_kinematics(*self.arm_2_positions)
                arm_1_xyz_position = np.array([0.0, self.arm_1_nominal[1], arm_1_fk[2]]) # We do this because otherwise the body y setponit of the arms may drift outwards
                arm_2_xyz_position = np.array([0.0, self.arm_2_nominal[1], arm_2_fk[2]])
                if self.first_state_loop:
                    self.contact_counter = 0
                    self.first_state_loop = False

                if self.retract_right < -1.5:
                    arm_1_goal = self.arm_1_nominal + np.array([0.0, 0.0, -0.2])
                    self.get_logger().info("Moving right arm to upwards position")
                    self.get_logger().info(f"Arm 1 goal position xyz: {arm_1_goal[0]:.3f}, {arm_1_goal[1]:.3f}, {arm_1_goal[2]:.3f} ")
                    self.get_logger().info(f"Arm 2 goal position xyz: {arm_2_xyz_position[0]:.2f}, {arm_2_xyz_position[1]:.2f}, {arm_2_xyz_position[2]:.2f}")
                    self.move_arms_to_xyz_position(arm_1_goal, arm_2_xyz_position)
                    self.xyz_setpoint1 = self.arm_1_nominal
                    self.retract_right = -1.0
                elif self.retract_left < -1.5:
                    arm_2_goal = self.arm_2_nominal + np.array([0.0, 0.0, -0.2])
                    self.get_logger().info("Moving left arm to upwards position")
                    self.move_arms_to_xyz_position(arm_1_xyz_position, arm_2_goal)
                    self.get_logger().info(f"Arm 1 goal position xyz: {arm_1_xyz_position[0]:.3f}, {arm_1_xyz_position[1]:.3f}, {arm_1_xyz_position[2]:.3f} ")
                    self.get_logger().info(f"Arm 2 goal position xyz: {arm_2_goal[0]:.2f}, {arm_2_goal[1]:.2f}, {arm_2_goal[2]:.2f}")
                    self.xyz_setpoint2 = self.arm_2_nominal
                    self.retract_left = -1.0

                # State transition
                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif self.landing_start_position is not None:
                    self.get_logger().info(f"[STABILIZE] LANDING POSITION: {self.landing_start_position.position[0]:.2F} {self.landing_start_position.position[1]:.2F} {self.landing_start_position.position[2]:.2F} \n Publish 2 to continue", throttle_duration_sec=1)
                    self.srv_set_servo_max_speed(0.5)
                    self.transition_state('wait_for_landing_cmd')
                elif (datetime.datetime.now() - self.state_start_time).seconds > 5 or self.input_state == 1:
                    self.transition_state('move_to_probing_location')

            case('wait_for_landing_cmd'):
                self.publishMDState(13)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint_contact, self.y_setpoint_contact, self.contact_altitude-0.5, self.heading_setpoint)
                self.move_arms_to_joint_position(
                    pi/3, 0.0, 1.6,
                    -pi/3, 0.0, -1.6)
                
                # State transition
                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')                
                if self.input_state == 2:
                    self.transition_state('pre_landing')              

            case('pre_landing'):
                self.publishMDState(21)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.landing_start_position.position[0], self.landing_start_position.position[1], self.landing_start_position.position[2], self.landing_start_position.yaw, yawspeed=0.2)                

                if self.first_state_loop:
                    self.first_state_loop = False

                if self.manipulator1_landing_position is not None and self.manipulator2_landing_position is not None:
                    arm_1_fk = self.position_forward_kinematics(*self.arm_1_positions)
                    arm_2_fk = self.position_forward_kinematics(*self.arm_2_positions)
                    self.get_logger().info(f'Moving manipulator to landing position in body frame:',throttle_duration_sec=1)
                    self.get_logger().info(f'Arm 1: {self.manipulator1_landing_position[0]:.2f}, {self.manipulator1_landing_position[1]:.2f}, {self.manipulator1_landing_position[2]:.2f}',throttle_duration_sec=1)
                    self.get_logger().info(f'Arm 2: {self.manipulator2_landing_position[0]:.2f}, {self.manipulator2_landing_position[1]:.2f}, {self.manipulator2_landing_position[2]:.2f}',throttle_duration_sec=1)
                    self.get_logger().info(f"Actual arm 1 positons: {arm_1_fk[0]:.2f} {arm_1_fk[1]:.2f} {arm_1_fk[2]:.2f}",throttle_duration_sec=1)
                    self.get_logger().info(f"Actual arm 2 positons: {arm_2_fk[0]:.2f} {arm_2_fk[1]:.2f} {arm_2_fk[2]:.2f}",throttle_duration_sec=1)

                    self.move_arms_to_xyz_position(self.manipulator1_landing_position, self.manipulator2_landing_position)

                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif ((datetime.datetime.now() - self.state_start_time).seconds > 8 or self.input_state == 1) and \
                    (self.manipulator1_landing_position is not None and self.manipulator2_landing_position is not None):
                    self.transition_state('land')
            
            case('land'):
                self.publishMDState(22)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.landing_start_position.position[0], self.landing_start_position.position[1], self.landing_start_position.position[2], self.landing_start_position.yaw, yawspeed=0.2)                
                self.landing_start_position.position[2] += self.landing_speed*self.timer_period
                self.get_logger().info(f"Altitude setpoint landing: {self.landing_start_position.position[2]:.3f} [m]",throttle_duration_sec=1)
                if self.first_state_loop:
                    self.get_logger().info(f"Landing!")
                    self.first_state_loop = False
                
                if not self.offboard and not self.dry_test:
                    self.transition_state('landed')
                if abs(self.landing_start_position.position[2]-self.vehicle_local_position.z) > 2.0 or self.input_state == 1:
                    self.transition_state('landed')

            case('landed'):
                self.publishMDState(23)
                self.get_logger().info('Done',throttle_duration_sec=1)
                self.disarmVehicle()

            case('emergency'):
                if self.first_state_loop:
                    self.srv_set_servo_max_speed(0.5)
                    self.first_state_loop = False
                self.move_arms_to_joint_position(
                    1.578, 0.0, -1.82,
                    -1.578, 0.0, 1.82)
                self.publishMDState(-1)
                self.get_logger().warn("Emergency state - no offboard mode", throttle_duration_sec=2)

            case(_):
                self.get_logger().error('State not recognized: {self.FSM_state}')
                self.transition_state('emergency')

    def srv_set_servo_mode(self, mode):
        # Set all servos to the specified mode (4 = continuous position, 1 = velocity)
        self.mode_set_req.operating_mode = mode
        self.future = self.servo_mode_client.call_async(self.mode_set_req)
        self.future.add_done_callback(self._set_mode_response_callback)

    def _set_mode_response_callback(self, future):
        self.future = future
        try:
            response = self.future.result()
            self.get_logger().info(f"Servo mode set: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def srv_set_servo_max_speed(self, max_speed):
        self.max_speed_set_req.max_speed = max_speed
        self.future_max_speed = self.servo_max_speed_client.call_async(self.max_speed_set_req)
        self.future_max_speed.add_done_callback(self._set_max_speed_response_callback)

    def _set_max_speed_response_callback(self, future):
        self.future_max_speed = future
        try:
            response = self.future_max_speed.result()
            self.get_logger().info(f"Servo max speed set: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")     

    def get_align_heading(self):
        """
        Get the heading that aligns the manipulator plane with the probing vector
        """
        # Normalize the probing direction vector and find horizontal projection
        normalized_probing_direction = self.probing_direction_body/np.linalg.norm(self.probing_direction_body)
        horizontal_projection = np.array([normalized_probing_direction[0], normalized_probing_direction[1]])
        # If the direction is straight up or down, return own heading
        if np.linalg.norm(horizontal_projection) < 1e-3:
            return self.vehicle_local_position.heading
        else:
            vehicle_y_in_world = np.array([np.sin(self.vehicle_local_position.heading), np.cos(self.vehicle_local_position.heading)])
            positive_heading = self.signed_angle_2d(vehicle_y_in_world, horizontal_projection)
            negative_heading = self.signed_angle_2d(-vehicle_y_in_world, horizontal_projection)
            if abs(positive_heading)>=abs(negative_heading):
                return negative_heading
            elif abs(negative_heading)>abs(positive_heading):
                return positive_heading

    def publish_arms_position_commands(self, q1_1, q2_1, q3_1, q1_2, q2_2, q3_2):
        msg = JointState()
        msg.position = [q1_1, q2_1, q3_1, q1_2, q2_2, q3_2]
        msg.velocity = [0., 0., 0., 0., 0., 0.]
        msg.name = ['q0', 'q1', 'q2', 'q3', 'q4', 'q5']
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_servo_state.publish(msg)

    def publish_arms_velocity_commands(self, q1_1, q2_1, q3_1, q1_2, q2_2, q3_2):
        msg = JointState()
        msg.position = [0., 0., 0., 0., 0., 0.]
        msg.velocity = [q1_1, q2_1, q3_1, q1_2, q2_2, q3_2]
        msg.name = ['q0', 'q1', 'q2', 'q3', 'q4', 'q5']
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_servo_state.publish(msg)

    def position_forward_kinematics(self, q_1, q_2, q_3):
        x_BS = -L_2*np.sin(q_2) - L_3*np.sin(q_2)*np.cos(q_3)
        y_BS = L_1*np.sin(q_1) + L_2*np.sin(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.cos(q_2)*np.cos(q_3) + L_3*np.sin(q_3)*np.cos(q_1)
        z_BS = -L_1*np.cos(q_1) - L_2*np.cos(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.sin(q_3) - L_3*np.cos(q_1)*np.cos(q_2)*np.cos(q_3)
        return [x_BS, y_BS, z_BS]

    def move_arms_to_xyz_position(self, target_position1, target_position2):
        msg = TwistStamped()
        msg.twist.linear.x = target_position1[0]
        msg.twist.linear.y = target_position1[1]
        msg.twist.linear.z = target_position1[2]
        msg.twist.angular.x = target_position2[0]
        msg.twist.angular.y = target_position2[1]
        msg.twist.angular.z = target_position2[2]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_manipulator_positions.publish(msg)

    def move_arms_in_xyz_velocity(self, target_velocity1, target_velocity2):
        msg = TwistStamped()
        msg.twist.linear.x = target_velocity1[0]
        msg.twist.linear.y = target_velocity1[1]
        msg.twist.linear.z = target_velocity1[2]
        msg.twist.angular.x = target_velocity2[0]
        msg.twist.angular.y = target_velocity2[1]
        msg.twist.angular.z = target_velocity2[2]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_manipulator_velocities.publish(msg)

    def move_arms_to_joint_position(self, q1_1, q2_1, q3_1, q1_2, q2_2, q3_2):
        q1_1_clipped = clip(q1_1, -pi, pi)
        q2_1_clipped = clip(q2_1, -pi/8., pi/8.)
        q3_1_clipped = clip(q3_1, -1.85, 1.85)
        q1_2_clipped = clip(q1_2, -pi, pi)
        q2_2_clipped = clip(q2_2, -pi/8., pi/8.)
        q3_2_clipped = clip(q3_2, -1.85, 1.85)
        self.publish_arms_position_commands(q1_1_clipped, q2_1_clipped, q3_1_clipped, q1_2_clipped, q2_2_clipped, q3_2_clipped)

    def publishMDState(self, state):
        msg = Int32()
        msg.data = state
        self.publisher_md_state.publish(msg)

    def publishOffboardPositionMode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publisher_offboard_control_mode.publish(msg)

    def publishTrajectoryPositionSetpoint(self, x, y, z, yaw, yawspeed=0.):
        # If clipping is not zero, clip the position
        if self.position_clip > 0.1:
            x_clipped = clip(x, -self.position_clip, self.position_clip)
            y_clipped = clip(y, -self.position_clip, self.position_clip)
            z_clipped = clip(z, -self.position_clip, 0.0) # Negative up
        else:
            x_clipped = x
            y_clipped = y
            z_clipped = z
        msg = TrajectorySetpoint()
        msg.position[0] = x_clipped
        msg.position[1] = y_clipped
        msg.position[2] = z_clipped
        msg.yaw = yaw
        msg.yawspeed=yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.publisher_vehicle_trajectory_setpoint.publish(msg)

    def armVehicle(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        #self.get_logger().info('Arm command sent')

    def disarmVehicle(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        #self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        #self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        #self.get_logger().info("Switching to land mode")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)

    # Callbacks
    def input_state_callback(self, msg):
        self.input_state = msg.data

    def vehicle_status_callback(self, msg):
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.armed = True
        else:
            self.armed = False

        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.offboard = True
        else:
            self.offboard = False
        
        if msg.latest_disarming_reason == VehicleStatus.ARM_DISARM_REASON_KILL_SWITCH:
            self.killed = True
        else:
            self.killed = False

        self.vehicle_status = msg

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def joint_states_callback(self, msg):
        self.arm_1_positions[0] = msg.position[0] # Pivot
        self.arm_1_positions[1] = msg.position[1] # Shoulder
        self.arm_1_positions[2] = msg.position[2] # Elbow

        self.arm_1_velocities[0] = msg.velocity[0] # Pivot
        self.arm_1_velocities[1] = msg.velocity[1] # Shoulder
        self.arm_1_velocities[2] = msg.velocity[2] # Elbow

        self.arm_1_effort[0] = msg.effort[0] # Pivot
        self.arm_1_effort[1] = msg.effort[1] # Shoulder
        self.arm_1_effort[2] = msg.effort[2] # Elbow

        self.arm_2_positions[0] = msg.position[3] # Pivot
        self.arm_2_positions[1] = msg.position[4] # Shoulder
        self.arm_2_positions[2] = msg.position[5] # Elbow

        self.arm_2_velocities[0] = msg.velocity[3] # Pivot
        self.arm_2_velocities[1] = msg.velocity[4] # Shoulder
        self.arm_2_velocities[2] = msg.velocity[5] # Elbow

        self.arm_2_effort[0] = msg.effort[3] # Pivot
        self.arm_2_effort[1] = msg.effort[4] # Shoulder
        self.arm_2_effort[2] = msg.effort[5] # Elbow

    def landing_point_callback(self, msg):
        self.get_logger().info(f"Received landing location!")
        self.landing_start_position = msg

    def landing_manipulator_callback(self, msg):
        self.manipulator1_landing_position = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.manipulator2_landing_position = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
    
    def contact_point_callback(self, msg):
        self.get_logger().info(f"Contact on arm {msg.data}")
        if msg.data == 1 and self.FSM_state == 'probing': # Right arm
            self.get_logger().info(f"Contact on arm {msg.data}")
            self.retract_right = -2.
            self.last_contact_time_right = datetime.datetime.now()
        elif msg.data == 2 and self.FSM_state == 'probing': # Left arm
            self.get_logger().info(f"Contact on arm {msg.data}")
            self.retract_left = -2.
            self.last_contact_time_left = datetime.datetime.now()
        else:
            self.get_logger().info(f"Contact detected but not in probing state")
        self.contact_counter += 1

    def Rx(self, angle:float, vector:np.array) -> np.array:
        return np.array([[1., 0., 0.],
                         [0., np.cos(angle), -np.sin(angle)],
                         [0., np.sin(angle), np.cos(angle)]]) @ vector

    def Rz(self,angle:float, vector:np.array) -> np.array:
        return np.array([[np.cos(angle), -np.sin(angle), 0.],
        [np.sin(angle), np.cos(angle), 0.],
        [0., 0., 1.]]) @ vector

    def R2(self, angle:float, vector:np.array) -> np.array:
        return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]) @ vector

    def signed_angle_2d(self, v1: np.array, v2: np.array) -> float:

        # Normalize
        v1_u = v1 / np.linalg.norm(v1)
        v2_u = v2 / np.linalg.norm(v2)

        # Dot and 2D cross
        dot = np.dot(v1_u, v2_u)
        cross = v1_u[0] * v2_u[1] - v1_u[1] * v2_u[0]

        return np.arctan2(cross, dot)

    def transition_state(self, new_state='end'):
        if self.input_state != 0:
            self.get_logger().info('Manually triggered state transition')
            self.input_state = 0
        self.state_start_time = datetime.datetime.now() # Reset start time for next state
        self.first_state_loop = True # Reset first loop flag
        self.get_logger().info(f"Transition from {self.FSM_state} to {new_state}")
        self.FSM_state = new_state
        self.counter = 0

def main():
    rclpy.init(args=None)
    mission_director_py = MissionDirectorPy()
    rclpy.spin(mission_director_py)

    # Destroy the node explicitly
    mission_director_py.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
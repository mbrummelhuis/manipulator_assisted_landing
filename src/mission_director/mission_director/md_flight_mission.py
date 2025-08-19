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

from feetech_ros2.srv import SetMode

L_1 = 0.110
L_2 = 0.317
L_3 = 0.330

class MissionDirectorPy(Node):
    def __init__(self):
        super().__init__('md_flight')

        # Parameters
        self.declare_parameter('frequency', 25.0)
        self.declare_parameter('position_clip', 0.0)
        self.declare_parameter('takeoff_altitude', -1.5)
        self.declare_parameter('probing_speed', 0.05)
        self.declare_parameter('probing_direction', [0., 0., 1.])

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

        # Manipulator interface
        self.publisher_manipulator_positions = self.create_publisher(TwistStamped, '/manipulator/in/position', 10)
        self.publisher_manipulator_velocities = self.create_publisher(TwistStamped, '/manipulator/in/velocity', 10)

        # Landing planner output
        self.subscriber_contact_point = self.create_subscription(Int32, '/contact/out/contact_point', self.contact_point_callback, 10)
        self.subscriber_landing_point = self.create_subscription(TrajectorySetpoint, '/landing/out/start_location', self.landing_point_callback, 10)
        self.subscriber_landing_manipulator = self.create_subscription(TwistStamped, '/landing/out/manipulator', self.landing_manipulator_callback, 10)

        # Mission director in/output
        self.subscriber_input_state = self.create_subscription(Int32, '/md/input', self.input_state_callback, 10)
        self.publisher_md_state = self.create_publisher(Int32, '/md/state', 10)

        # Set initial data
        self.FSM_state = 'entrypoint'
        self.dry_test = True
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
        self.retract_right = False
        self.retract_left = False
        self.last_contact_time_right = datetime.datetime.now()
        self.last_contact_time_left = datetime.datetime.now()
        self.most_recent_contact_point = None
        self.arm_1_positions = np.array([0.0, 0.0, 0.0])
        self.arm_1_velocities = np.array([0.0, 0.0, 0.0])
        self.arm_1_effort = np.array([0.0, 0.0, 0.0])
        self.arm_2_positions = np.array([0.0, 0.0, 0.0])
        self.arm_2_velocities = np.array([0.0, 0.0, 0.0])
        self.arm_2_effort = np.array([0.0, 0.0, 0.0])

        self.arm_1_nominal = np.array([0.0, 0.4, -0.1]) # Nominal XYZ posiiton in FRD body frame
        self.arm_2_nominal = np.array([0.0, -0.4, -0.1]) # Nominal XYZ posiiton in FRD body frame

        self.previous_ee_1 = self.arm_1_nominal
        self.previous_ee_2 = self.arm_2_nominal
        
        self.manipulator1_landing_position = None
        self.manipulator2_landing_position = None
        self.landing_start_position = None

        self.probing_direction_body = np.array(self.get_parameter('probing_direction').get_parameter_value().double_array_value)
        self.probing_speed = self.get_parameter('probing_speed').get_parameter_value().double_value
        self.get_logger().info(f'probing downward speed {self.probing_speed}')
        self.workspace_radius = L_1+L_2+L_3
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
                self.publishOffboardPositionMode()

                # State transition
                if (self.x_setpoint != 0.0 and self.y_setpoint != 0.0) or self.input_state == 1:
                    self.get_logger().info(f'Got position fix! \t x: {self.x_setpoint} \t y: {self.y_setpoint}')
                    self.transition_state(new_state='wait_for_servo_driver')

            case('wait_for_servo_driver'):
                self.publishMDState(1)
                # State transition
                if (datetime.datetime.now() - self.state_start_time).seconds > 5 or self.input_state == 1:
                    self.transition_state('default_config')

            case('default_config'):
                self.move_arms_to_joint_position(
                    pi/3, 0.0, -1.6,
                    -pi/3, 0.0, 1.6)
                self.publishMDState(2)
                if self.first_state_loop:
                    self.get_logger().info('Default config -- Suspend drone and continue')
                    self.first_state_loop = False

                if (datetime.datetime.now() - self.state_start_time).seconds > 3 or self.input_state == 1:
                    self.transition_state('sim_arm_offboard') # TODO switch to wait for arm offboard in real flight

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

            case('sim_arm_offboard'):
                if not self.offboard and self.counter%self.frequency==0:
                    #self.get_logger().info("Sending offboard command")
                    self.engage_offboard_mode()
                    self.counter = 0
                if not self.armed and self.offboard and self.counter%self.frequency==0:
                    #self.get_logger().info("Sending arm command")
                    self.armVehicle()
                    self.counter = 0

                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)

                self.publishMDState(3)
                if self.armed and self.offboard:
                    self.transition_state('takeoff')

            case('takeoff'): # Takeoff - wait for takeoff altitude
                self.publishMDState(4)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)
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
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.get_align_heading())

                if self.first_state_loop:
                    # Get the angle w.r.t. [0., 0., 1.] to align the arms
                    probing_direction_angle = self.signed_angle_2d(np.array([0., 1.]), np.array([np.linalg.norm(self.probing_direction_body[0:2]), self.probing_direction_body[2]]))
                    rotated_arm1_nominal = self.Rx(probing_direction_angle, self.arm_1_nominal)
                    rotated_arm2_nominal = self.Rx(probing_direction_angle, self.arm_2_nominal)

                    self.move_arms_to_xyz_position(rotated_arm1_nominal, rotated_arm2_nominal)
                    self.first_state_loop = False

                # State transition
                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif (datetime.datetime.now() - self.state_start_time).seconds > 10 or self.input_state == 1:
                    self.transition_state('probing')  

            case('switch_to_velocity_mode'): # TODO transition here from previous state in real flight (sim has no change mode service)
                self.publishMDState(10)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.get_align_heading())

                if self.first_state_loop:
                    self.first_state_loop = False
                    self.srv_set_servo_mode(1)

                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif self.input_state == 1 or self.future.result().success:
                    self.transition_state('probing')
            
            # Verify after here
            case('probing'):
                self.publishMDState(11)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.get_align_heading())

                if self.first_state_loop:
                    self.contact_counter = 0
                    self.first_state_loop = False
                    self.landing_start_position = None # Reset landing start position 

                # Retract if indicated, otherwise probe in forward direction, stop arm if nearing workspace edge
                velocity_vector_body = self.probing_direction_body*self.probing_speed
                if not self.retract_right and not self.retract_left:
                    self.move_arms_in_xyz_velocity(velocity_vector_body, velocity_vector_body)
                elif not self.retract_right and self.retract_left:
                    self.move_arms_in_xyz_velocity(velocity_vector_body, -velocity_vector_body)
                elif self.retract_right and not self.retract_left:
                    self.move_arms_in_xyz_velocity(-velocity_vector_body, velocity_vector_body)
                elif self.retract_right and self.retract_left:
                    self.move_arms_in_xyz_velocity(-velocity_vector_body, -velocity_vector_body)
                elif np.linalg.norm(self.position_forward_kinematics(self.arm_1_positions)) > self.workspace_radius-0.1:
                    self.move_arms_in_xyz_velocity(np.zeros(3), velocity_vector_body)
                elif np.linalg.norm(self.position_forward_kinematics(self.arm_2_positions)) > self.workspace_radius-0.1:
                    self.move_arms_in_xyz_velocity(velocity_vector_body, np.zeros(3))
                elif np.linalg.norm(self.position_forward_kinematics(self.arm_1_positions)) > self.workspace_radius-0.1 and np.linalg.norm(self.position_forward_kinematics(self.arm_2_positions)) > self.workspace_radius-0.1:
                    self.move_arms_in_xyz_velocity(np.zeros(3), np.zeros(3))


                # Conditions for stopping retraction after contact (2 seconds)
                if (datetime.datetime.now() - self.last_contact_time_right).seconds > 2.:
                    self.retract_right = False
                if (datetime.datetime.now() - self.last_contact_time_left).seconds > 2.:
                    self.retract_left = False

                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif self.landing_start_position is not None or self.input_state == 1:
                    self.transition_state('switch_to_position_mode')

            case('switch_to_position_mode'):
                self.publishMDState(20)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.get_align_heading())

                if self.first_state_loop:
                    self.first_state_loop = False
                    self.srv_set_servo_mode(4)

                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif self.input_state == 1 and self.future.result().success:
                    self.transition_state('probing')

            case('pre_landing'):
                self.publishMDState(21)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.landing_start_position.x, self.landing_start_position.y, self.landing_start_position.z, self.landing_start_position.heading)                

                if self.first_state_loop:
                    self.first_state_loop = False

                if self.manipulator1_landing_position is not None and self.manipulator2_landing_position is not None:
                    self.get_logger().info(f'Moving manipulator to landing position in body frame:')
                    self.get_logger().info(f'Arm 1: {self.manipulator1_landing_position[0]:.2f}, {self.manipulator1_landing_position[1]:.2f}, {self.manipulator1_landing_position[2]:.2f}')
                    self.get_logger().info(f'Arm 2: {self.manipulator2_landing_position[0]:.2f}, {self.manipulator2_landing_position[1]:.2f}, {self.manipulator2_landing_position[2]:.2f}')

                    self.move_arms_to_xyz_position(self.manipulator1_landing_position, self.manipulator2_landing_position)

                if not self.offboard and not self.dry_test:
                    self.transition_state('emergency')
                elif ((datetime.datetime.now() - self.state_start_time).seconds > 5 or self.input_state == 1) and \
                    (self.manipulator1_landing_position is not None and self.manipulator2_landing_position is not None):
                    self.transition_state('land')
            
            case('land'):
                self.publishMDState(22)
                self.land()
                if (datetime.datetime.now() - self.state_start_time).seconds > 5 or self.input_state == 1:
                    self.transition_state('landed')

            case('landed'):
                self.publishMDState(23)
                self.get_logger().info('Done')
                self.disarmVehicle()

            case('emergency'):
                self.move_arms_to_joint_position(
                    1.578, 0.0, -1.85,
                    -1.578, 0.0, 1.85)
                self.publishMDState(-1)
                if self.counter% (2*self.frequency) == 0: # Publish message every 2 seconds
                    self.get_logger().warn("Emergency state - no offboard mode")
                    self.counter = 0
                self.counter +=1

            case(_):
                self.get_logger().error('State not recognized: {self.FSM_state}')
                self.transition_state('emergency')

    def get_align_heading(self):
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
        msg.name = ['q1_1', 'q2_1', 'q3_1', 'q1_2', 'q2_2', 'q3_2']
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_servo_state.publish(msg)

    def publish_arms_velocity_commands(self, q1_1, q2_1, q3_1, q1_2, q2_2, q3_2):
        msg = JointState()
        msg.position = [0., 0., 0., 0., 0., 0.]
        msg.velocity = [q1_1, q2_1, q3_1, q1_2, q2_2, q3_2]
        msg.name = ['q1_1', 'q2_1', 'q3_1', 'q1_2', 'q2_2', 'q3_2']
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
        self.landing_start_position = msg

    def landing_manipulator_callback(self, msg):
        self.manipulator1_landing_position = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.manipulator2_landing_position = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
    
    def contact_point_callback(self, msg):
        if msg.data == 1: # Right arm
            self.retract_right = True
            self.last_contact_time_right = datetime.datetime.now()
        elif msg.data == 2: # Left arm
            self.retract_left = True
            self.last_contact_time_left = datetime.datetime.now()
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

    def signed_angle_2d(v1: np.array, v2: np.array) -> float:
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
        self.first_state_loop = False # Reset first loop flag
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
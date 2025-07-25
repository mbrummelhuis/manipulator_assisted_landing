import rclpy
from rclpy.node import Node
import datetime

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from numpy import pi, clip
import numpy as np

from scipy.optimize import least_squares

from std_msgs.msg import Int32

from sensor_msgs.msg import JointState

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand

L_1 = 0.110
L_2 = 0.317
L_3 = 0.330

class MissionDirectorPy(Node):
    def __init__(self):
        super().__init__('md_sim')

        # Parameters
        self.declare_parameter('frequency', 25.0)
        self.declare_parameter('position_clip', 0.0)
        self.declare_parameter('takeoff_altitude', 0.4)
        self.declare_parameter('probing_speed', 0.1)
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

        # Mission director in/output
        self.subscriber_input_state = self.create_subscription(Int32, '/md/input', self.input_state_callback, 10)
        self.publisher_md_state = self.create_publisher(Int32, '/md/state', 10)

        # Set initial data
        self.FSM_state = 'entrypoint'
        self.first_state_loop = True
        self.input_state = 0
        self.position_clip = self.get_parameter('position_clip').get_parameter_value().double_value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        self.armed = False
        self.offboard = False
        self.killed = False
        self.x_setpoint = 0.0
        self.y_setpoint = 0.0
        self.arm_1_positions = [0.0, 0.0, 0.0]
        self.arm_1_velocities = [0.0, 0.0, 0.0]
        self.arm_1_effort = [0.0, 0.0, 0.0]
        self.arm_2_positions = [0.0, 0.0, 0.0]
        self.arm_2_velocities = [0.0, 0.0, 0.0]
        self.arm_2_effort = [0.0, 0.0, 0.0]

        self.probing_direction = self.get_parameter('probing_direction').get_parameter_value().double_array_value
        self.probing_speed = self.get_parameter('probing_speed').get_parameter_value().double_value

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
                if self.first_state_loop:
                    self.get_logger().info("Waiting for position fix")
                    self.first_state_loop = False

                self.publishMDState(0)
                self.x_setpoint = self.vehicle_local_position.x
                self.y_setpoint = self.vehicle_local_position.y
                self.publishOffboardPositionMode()
                if (self.x_setpoint != 0.0 and self.y_setpoint != 0.0) or self.input_state == 1:
                    self.get_logger().info(f'Got position fix! \t x: {self.x_setpoint} \t y: {self.y_setpoint}')
                    self.transition_state(new_state='wait_for_servo_driver')

            case('wait_for_servo_driver'):
                if (datetime.datetime.now() - self.state_start_time).seconds > 2 or self.input_state == 1:
                    self.transition_state('move_arm_landed')

            case('move_arm_landed'):
                self.move_arms_to_joint_position(
                    pi/2, 0.0, -1.85,
                    -pi/2, 0.0, 1.85)
                self.publishMDState(1)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)

                 # Wait 5 seconds until the arm is in position
                if (datetime.datetime.now() - self.state_start_time).seconds > 3 or self.input_state == 1:
                    self.transition_state(new_state='sim_arm_offboard')
                    
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
                self.move_arms_to_joint_position(
                    pi/2, 0.0, -1.85,
                    -pi/2, 0.0, 1.85)
                # get current vehicle altitude
                current_altitude = self.vehicle_local_position.z

                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)
                if self.first_state_loop:
                    self.get_logger().info(f'Vehicle local position heading: {self.vehicle_local_position.heading}')
                    self.get_logger().info(f'Takeoff altitude: {self.takeoff_altitude}')
                    self.first_state_loop = False
                
                # check if vehicle has reached takeoff altitude
                if abs(current_altitude)+0.1 > abs(self.takeoff_altitude) or self.input_state==1:
                    self.transition_state('arms_sensing_configuration')
            
            case('arms_sensing_configuration'):
                self.publishMDState(5)
                self.move_arms_to_joint_position(
                    2*pi/3, 0.0, pi/3,
                    -2*pi/3, 0.0, -pi/3)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)

                if (datetime.datetime.now() - self.state_start_time).seconds > 30 or self.input_state == 1:
                    self.transition_state('move_arms_for_landing')

            case('probing'):
                self.publishMDState(6)
                # Get current position
                current_ee_1 = self.position_forward_kinematics(*self.arm_1_positions)
                current_ee_2 = self.position_forward_kinematics(*self.arm_2_positions)

                # Propagate current position by probing velocity
                target_ee_1 = current_ee_1 + self.probing_direction*self.probing_speed*self.timer_period
                target_ee_2 = current_ee_2 + self.probing_direction*self.probing_speed*self.timer_period

                # Invert kinematics on target positions and move joints
                res = self.move_ee_to_body_position(*target_ee_1, *target_ee_2)

                # State switch logic
                if res == -1 or self.contact:
                    self.transition_state()

            case('move_arms_for_landing'):
                self.publishMDState(8)
                self.move_arms_to_joint_position(
                    pi/2, 0.0, -1.85,
                    -pi/2, 0.0, 1.85)
                self.publishOffboardPositionMode()
                self.publishTrajectoryPositionSetpoint(self.x_setpoint, self.y_setpoint, self.takeoff_altitude, self.vehicle_local_position.heading)

                if (datetime.datetime.now() - self.state_start_time).seconds > 3 or self.input_state == 1:
                    self.transition_state('land')
            
            case('land'):
                self.publishMDState(9)
                self.land()
                if (datetime.datetime.now() - self.state_start_time).seconds > 5 or self.input_state == 1:
                    self.transition_state('land')

            case('landed'):
                self.publishMDState(10)
                self.get_logger().info('Done')
                self.disarmVehicle()

    def publish_arms_position_commands(self, q1_1, q2_1, q3_1, q1_2, q2_2, q3_2):
        msg = JointState()
        msg.position = [q1_1, q2_1, q3_1, q1_2, q2_2, q3_2]
        msg.velocity = [0., 0., 0., 0., 0., 0.]
        msg.name = ['q1_1', 'q2_1', 'q3_1', 'q1_2', 'q2_2', 'q3_2']
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_servo_state.publish(msg)

    def publish_arms_velocity_commands(self, q1_1, q2_1, q3_1, q1_2, q2_2, q3_2):
        msg = JointState()
        msg.position = [0., 0., 0.]
        msg.velocity = [q1_1, q2_1, q3_1, q1_2, q2_2, q3_2]
        msg.name = ['q1_1', 'q2_1', 'q3_1', 'q1_2', 'q2_2', 'q3_2']
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_servo_state.publish(msg)

    def position_forward_kinematics(self, q_1, q_2, q_3):
        x_BS = -L_2*np.sin(q_2) - L_3*np.sin(q_2)*np.cos(q_3)
        y_BS = L_1*np.sin(q_1) + L_2*np.sin(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.cos(q_2)*np.cos(q_3) + L_3*np.sin(q_3)*np.cos(q_1)
        z_BS = -L_1*np.cos(q_1) - L_2*np.cos(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.sin(q_3) - L_3*np.cos(q_1)*np.cos(q_2)*np.cos(q_3)
        return [x_BS, y_BS, z_BS]

    def move_ee_to_body_position(self, x1, y1, z1, x2, y2, z2):
        joints_1 = self.manipulator_inverse_kinematics(x1, y1, z1, self.arm_1_positions[0], self.arm_1_positions[1], self.arm_1_positions[2])
        joints_2 = self.manipulator_inverse_kinematics(x2, y2, z2, self.arm_2_positions[0], self.arm_2_positions[1], self.arm_2_positions[2])
        if len(joints_1)==3 and len(joints_2)==3:
            self.move_arms_to_joint_position(*joints_1, *joints_2)
            return 0
        else:
            return -1

    # Do inverse kinematics on the manipulator using a least squares optimization
    def manipulator_inverse_kinematics(self, x_target, y_target, z_target, x_current, y_current, z_current):
        
        def ik_residuals(q):
            q1, q2, q3 = q

            x = -L_2*np.sin(q2) - L_3*np.sin(q2)*np.cos(q3)
            y = L_1*np.sin(q1) + L_2*np.sin(q1)*np.cos(q2) + \
                L_3*np.sin(q1)*np.cos(q2)*np.cos(q3) + \
                L_3*np.sin(q3)*np.cos(q1)
            z = -L_1*np.cos(q1) - L_2*np.cos(q1)*np.cos(q2) + \
                L_3*np.sin(q1)*np.sin(q3) - \
                L_3*np.cos(q1)*np.cos(q2)*np.cos(q3)

            return [x - x_target, y - y_target, z - z_target]

        initial_guess = [x_current, y_current, z_current]

        lower_bounds = [-np.pi, -np.pi/8., -np.pi/2.]
        upper_bounds = [ np.pi,  np.pi/8.,  np.pi/2.]

        result = least_squares(
            ik_residuals,
            initial_guess,
            bounds=(lower_bounds, upper_bounds)
        )

        # Output result
        if result.success:
            q1, q2, q3 = result.x
            print("Joint solution (q1, q2, q3):", result.x)
            return [q1, q2, q3]
        else:
            print("Optimization failed:", result.message)
            return [-1]

    def move_arms_to_joint_position(self, q1_1, q2_1, q3_1, q1_2, q2_2, q3_2):
        self.publish_arms_position_commands(q1_1, q2_1, q3_1, q1_2, q2_2, q3_2)

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
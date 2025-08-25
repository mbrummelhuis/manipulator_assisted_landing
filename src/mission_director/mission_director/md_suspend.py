import rclpy
from rclpy.node import Node
import datetime

from numpy import pi, clip
import numpy as np

from scipy.optimize import minimize

from std_msgs.msg import Int32 

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from feetech_ros2.srv import SetMode, SetMaxSpeed

L_1 = 0.110
L_2 = 0.317
L_3 = 0.330

class MissionDirectorPy(Node):
    def __init__(self):
        super().__init__('md_flight')

        # Parameters
        self.declare_parameter('frequency', 25.0)
        self.declare_parameter('probing_speed', 0.1)
        self.declare_parameter('probing_direction', [0., 0., 1.])

        # Servo interfacing
        self.publisher_servo_state = self.create_publisher(JointState, '/servo/in/state', 10)
        self.subscriber_joint_states = self.create_subscription(JointState, '/servo/out/state', self.joint_states_callback, 10)
        self.servo_mode_client = self.create_client(SetMode, '/set_servo_mode') # 1 is velocity, 4 is continuous position
        while not self.servo_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for servo set mode service')
        self.mode_set_req = SetMode.Request() # Set max speed in rad/s
        self.servo_max_speed_client = self.create_client(SetMaxSpeed, '/set_servo_max_speed')
        while not self.servo_max_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for servo set max speed service')
        self.max_speed_set_req = SetMaxSpeed.Request()

        # Mission director in/output
        self.subscriber_input_state = self.create_subscription(Int32, '/md/input', self.input_state_callback, 10)
        self.publisher_md_state = self.create_publisher(Int32, '/md/state', 10)

        # Contact detector
        self.subscriber_contact = self.create_subscription(Int32, '/contact/out/effort', self.contact_callback, 10)

        # Manipulator interface
        self.publisher_manipulator_positions = self.create_publisher(TwistStamped, '/manipulator/in/position', 10)
        self.publisher_manipulator_velocities = self.create_publisher(TwistStamped, '/manipulator/in/velocity', 10)

        # Set initial data
        self.FSM_state = 'entrypoint'
        self.first_state_loop = True
        self.input_state = 0
        self.killed = False
        self.x_setpoint = 0.0
        self.y_setpoint = 0.0
        self.arm_1_positions = np.array([0.0, 0.0, 0.0])
        self.arm_1_velocities = np.array([0.0, 0.0, 0.0])
        self.arm_1_effort = np.array([0.0, 0.0, 0.0])
        self.arm_2_positions = np.array([0.0, 0.0, 0.0])
        self.arm_2_velocities = np.array([0.0, 0.0, 0.0])
        self.arm_2_effort = np.array([0.0, 0.0, 0.0])

        self.arm_1_nominal = np.array([0.0, 0.4, -0.3]) # Nominal XYZ posiiton in FRD body frame
        self.arm_2_nominal = np.array([0.0, -0.4, -0.3]) # Nominal XYZ posiiton in FRD body frame

        self.previous_ee_1 = self.arm_1_nominal
        self.previous_ee_2 = self.arm_2_nominal

        self.probing_direction = np.array(self.get_parameter('probing_direction').get_parameter_value().double_array_value)
        self.probing_speed = self.get_parameter('probing_speed').get_parameter_value().double_value
        self.get_logger().info(f'probing downward speed {self.probing_speed}')
        self.workspace_radius = L_1+L_2+L_3
        self.get_logger().info(f'Maximum workspace radius {self.workspace_radius}')

        self.contact = 0
        self.contact_arm_1 = False
        self.contact_arm_2 = False

        self.state_start_time = datetime.datetime.now()
    

        # Timer -- always last
        self.counter = 0
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        match self.FSM_state:
            case('entrypoint'): # Entry point - wait for position fix
                if self.input_state == 1:
                    self.transition_state(new_state='default_config')

            case('default_config'):
                self.move_arms_to_joint_position(
                    pi/3, 0.0, 1.7,
                    -pi/3, 0.0, -1.7)
                self.publishMDState(1)
                if self.first_state_loop:
                    self.get_logger().info('Default config')
                    self.srv_set_servo_max_speed(0.5)
                    self.first_state_loop = False

                if self.input_state == 1:
                    self.srv_set_servo_max_speed(0.1)
                    self.transition_state('test_landing_config')
                    self.xyz_setpoint1 = self.position_forward_kinematics(*self.arm_1_positions)
                    self.xyz_setpoint2 = self.position_forward_kinematics(*self.arm_2_positions)

            case('probing_position'):
                self.move_arms_to_xyz_position(self.xyz_setpoint1, self.xyz_setpoint2)

                if np.linalg.norm(self.xyz_setpoint1) < self.workspace_radius:
                    self.xyz_setpoint1 += self.probing_speed*self.probing_direction*self.timer_period
                
                if np.linalg.norm(self.xyz_setpoint2) < self.workspace_radius:
                    self.xyz_setpoint2 += self.probing_speed*self.probing_direction*self.timer_period


                self.get_logger().info(f"Arm 1 XYZ setpoints: {self.xyz_setpoint1[0]:.2f}, {self.xyz_setpoint1[1]:.2f}, {self.xyz_setpoint1[2]:.2f}", throttle_duration_sec=1)
                self.get_logger().info(f"Arm 2 XYZ setpoints: {self.xyz_setpoint2[0]:.2f}, {self.xyz_setpoint2[1]:.2f}, {self.xyz_setpoint2[2]:.2f}", throttle_duration_sec=1)

                if self.input_state == 1:
                    self.transition_state('stop_servos')

            case('test_set_mode'):
                self.publishMDState(2)
                if self.first_state_loop:
                    self.get_logger().info('Testing servo set mode')
                    self.first_state_loop = False

                    self.srv_set_servo_mode(1)

                # If unsuccesfull, retry service call
                if (self.future.result() is not None and self.future.result().success is False):
                    self.get_logger().info("Retrying set mode service call")
                    self.srv_set_servo_mode(1)

                if self.input_state == 1 and self.future.result().success:
                    self.transition_state('test_velocity_ik')

            case('test_velocity_ik'):
                self.publishMDState(3)
                if self.first_state_loop:
                    self.get_logger().info('Testing servo velocity mode')
                    self.first_state_loop = False

                self.probing_vector = self.probing_speed * self.probing_direction
                self.get_logger().info(f'Probing vector XYZ FRD: {self.probing_vector}')
                self.move_arms_in_xyz_velocity(self.probing_vector, self.probing_vector)

                if self.input_state == 1:
                    self.publish_arms_velocity_commands(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    self.transition_state('test_set_position_mode')
            
            case('test_set_position_mode'):
                self.publishMDState(4)
                self.move_arms_to_xyz_position( # Start publishing position commands before changing mode to ensure manipulator does not go to 0
                    np.array([0.08, 0.45, 0.15]),
                    np.array([0.08, -0.45, 0.15]))                 
                if self.first_state_loop:
                    self.get_logger().info('Testing servo position mode switch')
                    self.srv_set_servo_mode(4)
                    self.first_state_loop = False
                
                # If unsuccesfull, retry service call
                if (self.future.result() is not None and self.future.result().success is False):
                    self.get_logger().info("Retrying set mode service call")
                    self.srv_set_servo_mode(4)

                if self.input_state == 1 and self.future.result().success:
                    self.transition_state('test_landing_config')
            
            case('test_landing_config'):
                self.publishMDState(5)
                if self.first_state_loop:
                    self.get_logger().info('Landing config')
                    self.first_state_loop = False
                self.move_arms_to_xyz_position(
                    np.array([0.12, 0.45, 0.3]),
                    np.array([0.12, -0.45, 0.3]))                 
                if self.input_state == 1:
                    self.transition_state('done')


            case('stop_servos'):
                self.publishMDState(4)
                if self.first_state_loop:
                    self.get_logger().info('Stop servos')
                    self.first_state_loop = False
                self.move_arms_to_joint_position(*self.arm_1_positions, *self.arm_2_positions)


    def srv_set_servo_mode(self, mode):
        # Set all servos to the specified mode (4 = continuous position, 1 = velocity)
        self.mode_set_req.operating_mode = mode
        self.future_mode = self.servo_mode_client.call_async(self.mode_set_req)
        self.future_mode.add_done_callback(self._set_mode_response_callback)

    def _set_mode_response_callback(self, future):
        self.future_mode = future
        try:
            response = self.future_mode.result()
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

    # Callbacks
    def input_state_callback(self, msg):
        self.input_state = msg.data

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

    def contact_callback(self,msg):
        self.contact = msg.data
        if self.contact == 1 and self.FSM_state=='probing_right':
            self.get_logger().info('MD Contact on right arm')
            self.contact_arm_1 = True
            self.contact = 0
        elif self.contact == 2 and self.FSM_state=='probing_left':
            self.get_logger().info('MD Contact on right arm')
            self.contact_arm_2 = True
            self.contact = 0

    def transition_state(self, new_state='end'):
        if self.input_state != 0:
            self.get_logger().info('Manually triggered state transition')
            self.input_state = 0
        self.contact_arm_1 = False
        self.contact_arm_2 = False
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
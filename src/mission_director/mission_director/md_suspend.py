import rclpy
from rclpy.node import Node
import datetime

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from numpy import pi, clip
import numpy as np

from scipy.optimize import minimize

from std_msgs.msg import Int32

from sensor_msgs.msg import JointState

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

        # Mission director in/output
        self.subscriber_input_state = self.create_subscription(Int32, '/md/input', self.input_state_callback, 10)
        self.publisher_md_state = self.create_publisher(Int32, '/md/state', 10)

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

        self.previous_ee_1 = np.array([0.0, 0.0, 0.0])
        self.previous_ee_2 = np.array([0.0, 0.0, 0.0])

        self.probing_direction = np.array(self.get_parameter('probing_direction').get_parameter_value().double_array_value)
        self.probing_speed = self.get_parameter('probing_speed').get_parameter_value().double_value
        self.get_logger().info(f'probing downward speed {self.probing_speed}')
        self.workspace_radius = L_1+L_2+L_3
        self.get_logger().info(f'Maximum workspace radius {self.workspace_radius}')

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
                if self.input_state == 1:
                    self.transition_state(new_state='wait_for_servo_driver')

            case('wait_for_servo_driver'):
                self.publishMDState(0)
                if self.first_state_loop:
                    self.get_logger().info('Waiting for servo driver')
                    self.first_state_loop = False
                if self.input_state == 1:
                    self.transition_state('default_config')

            case('default_config'):
                self.move_arms_to_joint_position(
                    pi/3, 0.0, -1.6,
                    -pi/3, 0.0, 1.6)
                self.publishMDState(1)
                if self.first_state_loop:
                    self.get_logger().info('Default config -- Suspend drone and continue')
                    self.first_state_loop = False

                if self.input_state == 1:
                    self.transition_state('pre_sensing_config')

            case('pre_sensing_config'):
                self.move_arms_to_joint_position(
                    pi/3, 0.0, 1.6,
                    -pi/3, 0.0, -1.6)
                self.publishMDState(2)

                if self.input_state == 1:
                    self.transition_state('pre_sensing_config_ik')

            case('pre_sensing_config_ik'):
                if self.first_state_loop:
                    self.move_arms_to_bodyxyz_position(*[0.0, 0.5, 0.0], *[0.0, -0.5, 0.0])
                    self.first_state_loop = False
                self.publishMDState(3)

                if self.input_state == 1:
                    self.transition_state('ik_1')

            case('ik_1'):
                if self.first_state_loop:
                    self.move_arms_to_bodyxyz_position(*[0.0, 0.5, 0.1], *[0.0, -0.5, 0.1])
                    self.first_state_loop = False
                self.publishMDState(3)

                if self.input_state == 1:
                    self.transition_state('ik_2')

            case('ik_2'):
                if self.first_state_loop:
                    self.move_arms_to_bodyxyz_position(*[0.0, 0.5, 0.2], *[0.0, -0.5, 0.2])
                    self.first_state_loop = False
                self.publishMDState(3)
                self.previous_ee_1 = np.array([0.0, 0.5, 0.2])
                self.previous_ee_2 = np.array([0.0, -0.5, 0.2])

                if self.input_state == 1:
                    self.transition_state('probing')

            case('probing'):
                # Propagate current position by probing velocity
                target_ee_1 = self.previous_ee_1 + self.probing_direction*self.probing_speed*self.timer_period
                target_ee_2 = self.previous_ee_2 #+ self.probing_direction*self.probing_speed*self.timer_period

                self.previous_ee_1 = target_ee_1
                self.previous_ee_2 = target_ee_2

                # Invert kinematics on target positions and move joints
                res = self.move_arms_to_bodyxyz_position(*target_ee_1, *target_ee_2)

                if np.linalg.norm(target_ee_1) > self.workspace_radius:
                    self.transition_state('default_config')

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

    def move_arms_to_bodyxyz_position(self, x1_target, y1_target, z1_target, x2_target, y2_target, z2_target):
        self.get_logger().info(f'------ ARM 1 -------')
        joints_1 = self.manipulator_inverse_kinematics(x1_target, y1_target, z1_target, self.arm_1_positions)
        self.get_logger().info(f'------ ARM 2 -------')
        joints_2 = self.manipulator_inverse_kinematics(x2_target, y2_target, z2_target, self.arm_2_positions)
        if len(joints_1)==3 and len(joints_2)==3:
            self.move_arms_to_joint_position(*joints_1, *joints_2)
            return 0
        else:
            return -1

    # Do inverse kinematics on the manipulator using a least squares optimization
    def manipulator_inverse_kinematics(self, x_target, y_target, z_target, current_joint_positions:np.array):
        
        def ik_objective(state:np.array, target_position:np.array, current_state:np.array):
            q_1 = state[0]
            q_2 = state[1]
            q_3 = state[2]
            x_BS = -L_2*np.sin(q_2) - L_3*np.sin(q_2)*np.cos(q_3)
            y_BS = L_1*np.sin(q_1) + L_2*np.sin(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.cos(q_2)*np.cos(q_3) + L_3*np.sin(q_3)*np.cos(q_1)
            z_BS = -L_1*np.cos(q_1) - L_2*np.cos(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.sin(q_3) - L_3*np.cos(q_1)*np.cos(q_2)*np.cos(q_3)
            position = np.array([x_BS, y_BS, z_BS])

            # Position error (Euclidean distance)
            pos_err = np.linalg.norm(position - target_position)

            error = pos_err**2
            regularization = 0.001 * np.linalg.norm(state - current_state)
            return error + regularization

        x0 = current_joint_positions

        lower_bounds = [-np.pi, -np.pi/8., -3*np.pi/4.]
        upper_bounds = [ np.pi,  np.pi/8.,  3*np.pi/4.]
        bounds = list(zip(lower_bounds, upper_bounds))

        target_position = np.array([x_target, y_target, z_target])
        result = minimize(
            fun=ik_objective,
            x0=x0,
            args=(target_position, current_joint_positions),
            bounds=bounds,
            method='SLSQP',
            options={'ftol': 1e-4, 'maxiter': 1000, 'disp': False}
            )

        # In case of convergence
        if result.success == True:
            self.get_logger().info(f'Current joint positions: {current_joint_positions}')
            self.get_logger().info(f'Computed joint positions: {result.x}')
            # Check if output correct
            position_result = self.position_forward_kinematics(*result.x)
            self.get_logger().info(f'Target positions: {target_position}')
            self.get_logger().info(f'Computed positions: {position_result}')

            scores = self.ik_objective_eval(result.x, target_position, current_joint_positions)
            self.get_logger().info(f'Kinematic error {scores[0]}, regularization {scores[1]}')

            error = np.linalg.norm(np.array(position_result)-np.array([x_target, y_target, z_target]))
            if error > 0.0001:
                self.get_logger().info(f"Ik failed with error {error}")
            else:
                self.get_logger().info(f"Joint positions {result.x} yields FK {position_result} with target {[x_target, y_target, z_target]}")
                pass
            q1, q2, q3 = result.x
            self.get_logger().info(f"Joint solution (q1, q2, q3): {result.x}")
            return np.array([q1, q2, q3])
        
        else:
            self.get_logger().info(f"Optimization failed: {result.message}, returning current joint positions")
            self.get_logger().info(f'{result.message}')
            return current_joint_positions

    def ik_objective_eval(self, state:np.array, target_position:np.array, current_state:np.array):
        q_1 = state[0]
        q_2 = state[1]
        q_3 = state[2]
        x_BS = -L_2*np.sin(q_2) - L_3*np.sin(q_2)*np.cos(q_3)
        y_BS = L_1*np.sin(q_1) + L_2*np.sin(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.cos(q_2)*np.cos(q_3) + L_3*np.sin(q_3)*np.cos(q_1)
        z_BS = -L_1*np.cos(q_1) - L_2*np.cos(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.sin(q_3) - L_3*np.cos(q_1)*np.cos(q_2)*np.cos(q_3)
        position = np.array([x_BS, y_BS, z_BS])

        # Position error (Euclidean distance)
        pos_err = np.linalg.norm(position - target_position)

        error = pos_err**2
        regularization = 0.001 * np.linalg.norm(state - current_state)
        return [error, regularization]
        
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
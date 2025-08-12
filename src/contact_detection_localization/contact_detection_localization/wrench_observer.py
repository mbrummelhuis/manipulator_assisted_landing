import rclpy
from rclpy.node import Node

import numpy as np
import datetime

from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from px4_msgs.msg import SensorCombined, ActuatorMotors

L_1 = 0.118 # TODO
L_2 = 0.326 # TODO
L_3 = 0.273 #0.330 with tactip # TODOs

class ExternalWrenchObserver(Node):
    def __init__(self):
        super().__init__('external_wrench_observer')

        # Parameters
        self.declare_parameter('gain_force', 1.0)
        self.declare_parameter('alpha_force', 1.0)
        self.declare_parameter('gain_torque', 1.0)
        self.declare_parameter('alpha_torque', 1.0)
        self.declare_parameter('force_contact_threshold', 1.0)

        # IMU
        self.subscriber_accel = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_callback, 10)
        self.sensor_state = None

        # Actuators
        self.subscriber_actuators = self.create_subscription(ActuatorMotors, '/fmu/out/actuator_motors', self.actuator_callback, 10)
        self.actuator_thrust = None
        
        # Publisher
        self.contact_publisher = self.create_publisher(Int32, '/contact/out/effort', 10)

        
        # Model parameters
        self.thrust_coefficient = 19.468 # Obtained through experimental data
        self.gain_force = self.get_parameter('gain_force').get_parameter_value().double_value * np.eye(3)
        self.model_mass = 3.8 # [kg]
        self.acceleration_gravity = np.array([0., 0., 9.81])
        self.linear_allocation_matrix = np.array([0., 0., 0., 0.],
                                                 [0., 0., 0., 0.],
                                                 [-1., -1., -1., -1.])
        self.alpha_force = self.get_parameter('alpha_force').get_parameter_value().double_value
        
        self.gain_torque = self.get_parameter('gain_torque').get_parameter_value().double_value * np.eye(3)
        self.inertia = np.array([0.071, -1.712e-5, -5.928e-6],
                                [-1.712e-5, 0.059, -1.448e-5],
                                [-5.928e-6, -1.448e-5, 0.121]) # [kgm2]
        
        
        arm_x = 0.184 # [m] Moment arm along the body x-axis
        arm_y = 0.231 # [m] Moment arm along the body y-axis
        drag_coeff = 0.05 # 
        self.rotational_allocation_matrix = np.array([-arm_y, arm_y, arm_y, -arm_y], # Roll moment
                                                     [arm_x, -arm_x, arm_x, -arm_x], # Pitch moment
                                                     [-drag_coeff, -drag_coeff, drag_coeff, drag_coeff]) # Yaw moment
        self.alpha_torque = self.get_parameter('alpha_torque').get_parameter_value().double_value

        # Contact detection and localization
        self.contact = False
        self.force_contact_threshold = self.get_parameter('force_contact_threshold').get_parameter_value().double_value

        # Timer -- always last
        self.previous_time = datetime.datetime.now()
        self.counter = 0
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.sensor_state and self.actuator_thrust:
            self.wrench_observer()
            self.contact_detection_localization()


    def wrench_observer(self):
        dt = datetime.datetime.now() - self.previous_time # Get time difference since last
        self.previous_time = datetime.datetime.now()

        # Force observer
        force_dot = self.gain_force @ (
            self.model_mass * self.sensor_acceleration + 
            self.model_mass * self.acceleration_gravity +
            self.linear_allocation_matrix @ self.actuator_state - 
            self.most_recent_force_estimate)

        self.most_recent_force_estimate = (self.alpha_force * (self.most_recent_force_estimate + dt * force_dot) +
            (1. - self.alpha_force)*self.most_recent_force_estimate)

        # Torque observer
        # J@omega
        current_angular_momentum = self.inertia @ self.sensor_angular_velocity

        # Model torque
        current_model_momentum = (np.cross(np.array(self.sensor_angular_velocity, np.matmul(self.inertia, self.sensor_angular_velocity))) - 
                                  self.rotational_allocation_matrix @ self.actuator_state) * dt

        # Torque estimate
        current_torque_estimate = current_angular_momentum + current_model_momentum + self.momentum_integral

        # Update integral
        self.momentum_integral = self.momentum_integral + current_model_momentum - self.most_recent_torque_estimate

        self.most_recent_torque_estimate = (self.alpha_torque * current_torque_estimate +
            (1. - self.alpha_torque) * self.most_recent_torque_estimate)
        
        self.get_logger().info(f'Force estimate: {self.most_recent_force_estimate:.2f} [N] \t Torque estimate {self.most_recent_torque_estimate:2f} [Nm]')

    def contact_detection_localization(self):
        # If norm of force estimate is high enough, assume contact
        if np.linalg.norm(self.most_recent_force_estimate) > self.force_contact_threshold:
            self.get_logger().info(f'Contact detected! Force magnitude: {np.linalg.norm(self.most_recent_force_estimate)}')
            self.contact = True
        else:
            self.contact = False

        if self.contact:
            # Determine the contact point
            pass

    def sensor_callback(self, msg):
        self.sensor_acceleration = np.array(msg.accelerometer_m_s2)
        self.sensor_angular_velocity = np.array(msg.gyro_rad)

    def actuator_callback(self, msg):
        self.actuator_thrust = np.array([msg.control[0], msg.control[1], msg.control[2], msg.control[3]]) * self.thrust_coefficient


    def positionForwardKinematics(self, q_1, q_2, q_3):
        x_BS = -L_2*np.sin(q_2) - L_3*np.sin(q_2)*np.cos(q_3)
        y_BS = L_1*np.sin(q_1) + L_2*np.sin(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.cos(q_2)*np.cos(q_3) + L_3*np.sin(q_3)*np.cos(q_1)
        z_BS = -L_1*np.cos(q_1) - L_2*np.cos(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.sin(q_3) - L_3*np.cos(q_1)*np.cos(q_2)*np.cos(q_3)
        return [x_BS, y_BS, z_BS]

def main():
    rclpy.init(args=None)
    wrench_observer = ExternalWrenchObserver()
    rclpy.spin(wrench_observer)

    # Destroy the node explicitly
    wrench_observer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
import datetime

from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from px4_msgs.msg import SensorCombined, ActuatorMotors

L_1 = 0.118 # TODO
L_2 = 0.326 # TODO
L_3 = 0.273 #0.330 with tactip # TODOs

class ExternalWrenchObserver(Node):
    def __init__(self):
        super().__init__('external_wrench_observer')

        # Parameters
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('gain_force', 1.0)
        self.declare_parameter('alpha_force', 1.0)
        self.declare_parameter('gain_torque', 1.0)
        self.declare_parameter('alpha_torque', 1.0)
        self.declare_parameter('alpha_angular_velocity', 1.0)
        self.declare_parameter('force_contact_threshold', 1.0)
        self.declare_parameter('contact_force_proximity_threshold', 0.1)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # IMU
        self.subscriber_accel = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_callback, qos_profile)
        self.sensor_acceleration = np.array([0., 0., 0.])
        self.sensor_angular_velocity = np.array([0., 0., 0.])

        # Actuators
        self.subscriber_actuators = self.create_subscription(ActuatorMotors, '/fmu/out/actuator_motors', self.actuator_callback, qos_profile)
        self.actuator_thrust = None

        self.subscriber_servos = self.create_subscription(JointState, '/servo/out/state', self.servo_callback, 10)
        self.servo_state = None

        # Publisher
        self.publisher_loa_direction = self.create_publisher(Vector3Stamped, '/contact/out/loa_direction', 10)
        self.publisher_loa_point = self.create_publisher(PointStamped, '/contact/out/loa_point', 10)
        self.publisher_estimator_force = self.create_publisher(Vector3Stamped, '/contact/out/estimated_force', 10)
        self.publisher_estimator_torque = self.create_publisher(Vector3Stamped, '/contact/out/estimated_torque', 10)
        self.publisher_contact_point_coords = self.create_publisher(PointStamped, '/contact/out/contact_point_coords', 10)
        self.publisher_contact_point = self.create_publisher(Int32, '/contact/out/contact_point',10)

        
        # Model parameters
        self.thrust_coefficient = 19.468 # Obtained through experimental data
        self.propeller_incline_angle = 5 # [deg] propeller incline in degreess
        self.gain_force = self.get_parameter('gain_force').get_parameter_value().double_value * np.eye(3)
        self.model_mass = 3.701 # [kg]
        self.acceleration_gravity = np.array([0., 0., 9.81])
        self.linear_allocation_matrix = np.array([[-np.sin(np.deg2rad(60.))*np.sin(np.deg2rad(self.propeller_incline_angle)), np.sin(np.deg2rad(60.))*np.sin(np.deg2rad(self.propeller_incline_angle)), -np.sin(np.deg2rad(60.))*np.sin(np.deg2rad(self.propeller_incline_angle)), np.sin(np.deg2rad(60.))*np.sin(np.deg2rad(self.propeller_incline_angle))],
                                                 [-np.sin(np.deg2rad(30.))*np.sin(np.deg2rad(self.propeller_incline_angle)), np.sin(np.deg2rad(30.))*np.sin(np.deg2rad(self.propeller_incline_angle)), np.sin(np.deg2rad(30.))*np.sin(np.deg2rad(self.propeller_incline_angle)), -np.sin(np.deg2rad(30.))*np.sin(np.deg2rad(self.propeller_incline_angle))],
                                                 [-np.cos(np.deg2rad(self.propeller_incline_angle)), -np.cos(np.deg2rad(self.propeller_incline_angle)), -np.cos(np.deg2rad(self.propeller_incline_angle)), -np.cos(np.deg2rad(self.propeller_incline_angle))]])
        self.alpha_force = self.get_parameter('alpha_force').get_parameter_value().double_value

        self.gain_torque = self.get_parameter('gain_torque').get_parameter_value().double_value * np.eye(3)
        self.inertia = np.array([[0.071, -1.712e-5, -5.928e-6],
                                [-1.712e-5, 0.059, -1.448e-5],
                                [-5.928e-6, -1.448e-5, 0.121]]) # [kgm2]

        arm_x = 0.184 # [m] Moment arm along the body x-axis
        arm_y = 0.231 # [m] Moment arm along the body y-axis
        drag_coeff = 0.1e-5 # Somewhat arbitrary
        yaw_moment_arm = 0.33911 # [m] Moment arm of yaw contribution of thrust
        self.rotational_allocation_matrix = np.array([[-arm_y*np.cos(np.deg2rad(self.propeller_incline_angle)), arm_y*np.cos(np.deg2rad(self.propeller_incline_angle)), arm_y*np.cos(np.deg2rad(self.propeller_incline_angle)), -arm_y*np.cos(np.deg2rad(self.propeller_incline_angle))], # Roll moment
                                                     [arm_x*np.cos(np.deg2rad(self.propeller_incline_angle)), -arm_x*np.cos(np.deg2rad(self.propeller_incline_angle)), arm_x*np.cos(np.deg2rad(self.propeller_incline_angle)), -arm_x*np.cos(np.deg2rad(self.propeller_incline_angle))], # Pitch moment
                                                     [np.sin(np.deg2rad(self.propeller_incline_angle))*yaw_moment_arm+drag_coeff, np.sin(np.deg2rad(self.propeller_incline_angle))*yaw_moment_arm+drag_coeff, -np.sin(np.deg2rad(self.propeller_incline_angle))*yaw_moment_arm+drag_coeff, -np.sin(np.deg2rad(self.propeller_incline_angle))*yaw_moment_arm+drag_coeff]]) # Yaw moment
        self.alpha_torque = self.get_parameter('alpha_torque').get_parameter_value().double_value

        self.R_accelerometer = np.array([[-1., 0., 0.],
                                         [0., -1., 0.],
                                         [0., 0., -1.]])

        # Initial values for updating member variables
        self.most_recent_force_estimate = np.array([0., 0., 0.])
        self.most_recent_torque_estimate = np.array([0., 0., 0.])
        self.momentum_integral = np.array([0., 0., 0.])
        self.alpha_angular_velocity = self.get_parameter('alpha_angular_velocity').get_parameter_value().double_value

        # Contact detection and localization
        self.contact = False
        self.force_contact_threshold = self.get_parameter('force_contact_threshold').get_parameter_value().double_value
        self.contact_point_proximity_threshold = self.get_parameter('contact_force_proximity_threshold').get_parameter_value().double_value

        # Candidate contact points
        leg_x = 0.17 # [m] Leg contact point distance along body x-axis
        leg_y = 0.078 # [m] Leg contact point distance along body y-axis
        leg_z = 0.09 # [m] Leg contact point distance along body z-axis
        self.contact_point_candidates = {
            'right_arm': {'coords': np.array([0.0, 0.0, 0.0]), 'index': 1},
            'left_arm': {'coords': np.array([0.0, 0.0, 0.0]), 'index': 2},
            'right_front_leg': {'coords': np.array([leg_x, leg_y, leg_z]), 'index': 3},
            'right_back_leg': {'coords': np.array([-leg_x, leg_y, leg_z]), 'index': 4},
            'left_back_leg': {'coords': np.array([-leg_x, -leg_y, leg_z]), 'index': 5},
            'left_front_leg': {'coords': np.array([leg_x, -leg_y, leg_z]), 'index': 6},
            'center': {'coords': np.array([0., 0., leg_z]), 'index': 9}
        }
        # Timer -- always last
        self.previous_time = datetime.datetime.now()
        self.counter = 0
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        #self.get_logger().info(f'Running wrench observer. {self.sensor_acceleration}, {self.sensor_angular_velocity}, {self.actuator_thrust}')
        if self.sensor_acceleration is None or self.sensor_angular_velocity is None or self.actuator_thrust is None:
            return
        else:
            #self.get_logger().info(f'Passed initialization check')
            self.wrench_observer()
            self.contact_detection_localization()
            return

    def wrench_observer(self):
        dt = (datetime.datetime.now() - self.previous_time).total_seconds() # Get time difference since last
        # self.get_logger().info(f'dt [sec]: {dt}')
        self.previous_time = datetime.datetime.now()

        # Force observer
        force_dot = self.gain_force @ ( # TODO rotate 
            self.model_mass * self.sensor_acceleration - # Add gravity here in the math in the paper
            self.linear_allocation_matrix @ self.actuator_thrust - 
            self.most_recent_force_estimate)

        # Update force by propagating force_dot and EWMA smoothing
        self.most_recent_force_estimate = (self.alpha_force * (self.most_recent_force_estimate + dt * force_dot) +
            (1. - self.alpha_force)*self.most_recent_force_estimate)
        self.publish_estimated_force(self.most_recent_force_estimate)

        # Torque observer
        # Calculate observed angular momentum
        current_angular_momentum = self.inertia @ self.sensor_angular_velocity
        # Update angular momentum model
        self.momentum_integral = self.momentum_integral + (-np.cross(self.sensor_angular_velocity, current_angular_momentum) + 
                                                           self.rotational_allocation_matrix @ self.actuator_thrust + self.most_recent_torque_estimate)
        # Torque estimate is the difference between measured and model, with a gain
        current_torque_estimate = self.gain_torque @ (current_angular_momentum - self.momentum_integral)
        
        # self.get_logger().info(f'Actuator moments: {(self.rotational_allocation_matrix @ self.actuator_thrust)[0]:.2f}, {(self.rotational_allocation_matrix @ self.actuator_thrust)[1]:.2f}, {(self.rotational_allocation_matrix @ self.actuator_thrust)[2]:.2f}')

        # EWMA smoothing
        self.most_recent_torque_estimate = (self.alpha_torque * current_torque_estimate +
            (1. - self.alpha_torque) * self.most_recent_torque_estimate)
        self.publish_estimated_torque(self.most_recent_torque_estimate)
        self.get_logger().info(f'Force estimate: [{self.most_recent_force_estimate[0]:.2f}, {self.most_recent_force_estimate[1]:.2f}, {self.most_recent_force_estimate[2]:.2f}][N]', throttle_duration_sec=1)
        self.get_logger().info(f'Torque estimate [{self.most_recent_torque_estimate[0]:.2f}, {self.most_recent_torque_estimate[1]:.2f}, {self.most_recent_torque_estimate[2]:.2f}] [Nm]', throttle_duration_sec=1)

    def contact_detection_localization(self):
        # If norm of force estimate is high enough, assume contact
        if np.linalg.norm(self.most_recent_force_estimate) > self.force_contact_threshold:
            self.get_logger().info(f'Contact detected! Force magnitude: {np.linalg.norm(self.most_recent_force_estimate)}', throttle_duration_sec=1)
            self.contact = True
        else:
            self.contact = False

        if self.contact:
            point_on_line, direction, success = self.compute_line_of_action()
            if success:
                self.select_contact_point(point_on_line, direction)
            pass

    def compute_line_of_action(self):
        """
        Compute the line of action of an external contact force in the body frame
        given force and moment residuals.
                
        Returns:
            r0 : (3,) np.array
                A particular point on the line of action
            direction : (3,) np.array
                Unit direction vector of the line (parallel to Delta_F)
            valid : bool
                Whether the line is well-defined (False if |Delta_F| is tiny)
        """
        F_norm = np.linalg.norm(self.most_recent_force_estimate)
        if F_norm < 1e-6:
            # Force too small to define a reliable line of action
            return None, None, False

        # Normalise force vector
        direction = self.most_recent_force_estimate / F_norm

        # Particular point on the line closest to r_CoM
        point_on_line = np.cross(self.most_recent_torque_estimate, self.most_recent_force_estimate) / (F_norm**2)

        #self.get_logger().info(f'LOA found! Point: [{point_on_line[0]:.2f}, {point_on_line[1]:.2f}, {point_on_line[2]:.2f}], dir [{direction[0]:.2f}, {direction[1]:.2f}, {direction[2]:.2f}]')
        self.publish_loa_direction(direction)
        self.publish_loa_point(point_on_line)
        return point_on_line, direction, True
    
    def select_contact_point(self, point_on_line, direction_vector):
        if self.servo_state is not None:
            self.update_ee_locations()
        else:
            self.get_logger().warn(f'No servo state, cannot compute forward kinematics!', throttle_duration_sec=1)
            return None, np.inf
        best_point = None
        best_distance = np.inf

        for point_name, data in self.contact_point_candidates.items():
            diff = data['coords'] - point_on_line
            dist = np.linalg.norm(np.cross(diff, direction_vector))  # perpendicular distance
            if dist < best_distance:
                best_distance = dist
                best_point = point_name

        if best_distance < self.contact_point_proximity_threshold:
            #self.get_logger().info(f'Found contact point: {point_name}')
            self.publish_contact_point_coords(self.contact_point_candidates[point_name]['coords'])
            self.publish_contact_point(self.contact_point_candidates[point_name]['index'])
            return best_point, best_distance
        else:
            self.get_logger().info(f'No candidate point within proximity. Distance: {best_distance}, threshold: {self.contact_point_proximity_threshold}')
            return None, best_distance

    def update_ee_locations(self):
        FK_right = self.position_forward_kinematics(self.servo_state.position[0], self.servo_state.position[1], self.servo_state.position[2])
        FK_left = self.position_forward_kinematics(self.servo_state.position[3], self.servo_state.position[4], self.servo_state.position[5])
        self.contact_point_candidates['right_arm']['coords'] = np.array(FK_right)
        self.contact_point_candidates['left_arm']['coords'] = np.array(FK_left)

    def publish_loa_direction(self, direction):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = direction[0]
        msg.vector.y = direction[1]
        msg.vector.z = direction[2]
        self.publisher_loa_direction.publish(msg)

    def publish_loa_point(self, point):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = point[0]
        msg.point.y = point[1]
        msg.point.z = point[2]
        self.publisher_loa_point.publish(msg)

    def publish_estimated_force(self, force):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = force[0]
        msg.vector.y = force[1]
        msg.vector.z = force[2]
        self.publisher_estimator_force.publish(msg)

    def publish_estimated_torque(self, torque):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = torque[0]
        msg.vector.y = torque[1]
        msg.vector.z = torque[2]
        self.publisher_estimator_torque.publish(msg)

    def publish_contact_point_coords(self, point):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = point[0]
        msg.point.y = point[1]
        msg.point.z = point[2]
        self.publisher_contact_point_coords.publish(msg)

    def publish_contact_point(self, int):
        msg = Int32()
        msg.data = int
        self.publisher_contact_point.publish(msg)

    def sensor_callback(self, msg):
        self.sensor_acceleration = np.array(msg.accelerometer_m_s2)
        self.sensor_angular_velocity = self.alpha_angular_velocity * np.array(msg.gyro_rad) + (1.-self.alpha_angular_velocity)*self.sensor_angular_velocity

    def actuator_callback(self, msg):
        self.actuator_thrust = np.array([msg.control[0], msg.control[1], msg.control[2], msg.control[3]]) * self.thrust_coefficient

    def servo_callback(self, msg):
        self.servo_state = msg

    def position_forward_kinematics(self, q_1, q_2, q_3):
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
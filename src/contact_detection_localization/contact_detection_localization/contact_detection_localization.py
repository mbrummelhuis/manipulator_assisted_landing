import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from px4_msgs.msg import SensorCombined

L_1 = 0.110
L_2 = 0.317
L_3 = 0.330

class ContactDetectionLocalization(Node):
    def __init__(self):
        super().__init__('contact_detection')

        self.declare_parameter('effort_threshold', 1.0)
        self.declare_parameter('acc_threshold', 1.0)

        #self.subscriber_accel = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_callback, 10)
        
        # Servo subscriber
        self.subscriber_joints = self.create_subscription(JointState, '/servo/out/state', self.servo_callback, 10)
        self.joint_state = None
        self.effort_diff = np.zeros(6)

        self.previous_accelero_magnitude = 0.0
        self.contact_acc = False
        self.effort_threshold = self.get_parameter('effort_threshold').get_parameter_value().double_value
        self.acc_threshold = self.get_parameter('acc_threshold').get_parameter_value().double_value

        self.contact_point_1 = None
        self.contact_point_2 = None

        self.contact_publisher = self.create_publisher(Int32, '/contact/out/effort', 10)

    def servo_callback(self, msg):
        # Calculate torque difference per joint:
        # If the first message, only populate the member variable
        if self.joint_state == None:
            self.joint_state = msg
            return
        
        # For each joint, check if the effort is above the threshold
        for i in range(len(6)):
            self.effort_diff[i] = self.joint_state.effort[i] - msg.effort[i]
            if abs(self.effort_diff[i]) > self.effort_threshold and i < 3:
                self.get_logger().info(f'Contact detected on arm 1! Effort diff {self.effort_diff[i]}')
            elif abs(self.effort_diff[i]) > self.effort_threshold and i >= 3:
                self.get_logger().info(f'Contact detected on arm 2! Effort diff {self.effort_diff[i]}')
            
        return



    def estimateExternalWrench(self, body_accleration:np.array, body_angl_velocity:np.array, R_BI:np.array, motor_cmds, 
                               mass, inertia, motor_allocation, thrust_coeffs, g=np.array([0, 0, 9.81])):
        """
        Momentum-based wrench observer for UAV contact detection.
        
        Args:
            accel_body: (3,) np.array, linear acceleration from IMU (body frame)
            omega_body: (3,) np.array, angular velocity from IMU (body frame)
            R_BI: (3,3) np.array, rotation matrix body->world
            motor_cmds: (N,) np.array, motor commands (e.g., PWM normalized [0..1])
            mass: float, UAV mass
            inertia: (3,3) np.array, UAV inertia tensor in body frame
            motor_allocation: (4, N) np.array, allocation matrix mapping motor thrusts to [Fx,Fy,Fz,Mx,My,Mz]
            thrust_coeffs: (N,) np.array, thrust coefficients per motor
            g: (3,) np.array, gravity in world frame
            prev_omega: (3,) np.array, angular velocity at previous step (optional, for alpha estimate)
            dt: float, timestep (required if prev_omega is given)
            
        Returns:
            Delta_F: (3,) np.array, external force residual in body frame
            Delta_M: (3,) np.array, external moment residual in body frame
        """
        # Step 1: Compute linear acceleration of CoM in body frame
        a_body = body_accleration - R_BI.T @ g  # Correct for stationary gravity (downward positive)
        
        # Step 2: Compute angular acceleration
        if self.prev_angl_velocity is not None and self.dt is not None:
            alpha_body = (body_angl_velocity - self.prev_angl_velocity) / self.dt
        else:
            alpha_body = np.zeros(3)  # fallback if derivative not available
        
        # Step 3: Predict propulsive forces & moments
        motor_thrusts = thrust_coeffs * motor_cmds
        # motor_allocation maps thrusts to [Fx,Fy,Fz,Mx,My,Mz]
        wrench_pred = motor_allocation @ motor_thrusts
        F_prop = wrench_pred[:3]
        M_prop = wrench_pred[3:]
        
        # Step 4: Compute measured force & moment from dynamics
        F_meas = mass * a_body
        M_meas = inertia @ alpha_body + np.cross(self.prev_angl_velocity, inertia @ self.prev_angl_velocity)
        
        # Step 5: Residuals
        Delta_F = F_meas - (mass * R_BI.T @ g + F_prop)
        Delta_M = M_meas - M_prop
        
        return Delta_F, Delta_M

    def computeLineOfAction(self, Delta_F, Delta_M, r_CoM):
        """
        Compute the line of action of an external contact force given
        force and moment residuals (Delta_F, Delta_M).
        
        Args:
            Delta_F : (3,) np.array
                External force residual in body frame
            Delta_M : (3,) np.array
                External moment residual in body frame (about r_CoM)
            r_CoM : (3,) np.array
                Center of mass location in body frame (usually [0,0,0])
                
        Returns:
            r0 : (3,) np.array
                A particular point on the line of action
            direction : (3,) np.array
                Unit direction vector of the line (parallel to Delta_F)
            valid : bool
                Whether the line is well-defined (False if |Delta_F| is tiny)
        """
        F_norm = np.linalg.norm(Delta_F)
        if F_norm < 1e-6:
            # Force too small to define a reliable line of action
            return None, None, False

        direction = Delta_F / F_norm

        # Particular point on the line closest to r_CoM
        r0 = r_CoM + np.cross(Delta_M, Delta_F) / (F_norm**2)
        
        return r0, direction, True
    
    def selectContactPoint(self, r0, direction_vector):
        best_point = None
        best_distance = np.inf

        for r_i in self.contact_point_candidates:
            diff = r_i - r0
            dist = np.linalg.norm(np.cross(diff, direction_vector))  # perpendicular distance
            if dist < best_distance:
                best_distance = dist
                best_point = r_i

        if best_distance < self.contact_point_proximity_threshold:
            return best_point, best_distance
        else:
            return None, best_distance


    def sensor_callback(self, msg):
        # start condition
        if self.previous_accelero_magnitude == 0.0:
            self.previous_accelero_magnitude = np.linalg.norm(np.array(msg.accelerometer_m_s2))
            return

        # filter linear acceleration to detect contact
        accelero_magnitude = np.linalg.norm(np.array(msg.accelerometer_m_s2))
        accelero_difference = accelero_magnitude - self.previous_accelero_magnitude
        if accelero_difference > self.acc_threshold:
            self.contact_acc = True
            return

    def positionForwardKinematics(self, q_1, q_2, q_3):
        x_BS = -L_2*np.sin(q_2) - L_3*np.sin(q_2)*np.cos(q_3)
        y_BS = L_1*np.sin(q_1) + L_2*np.sin(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.cos(q_2)*np.cos(q_3) + L_3*np.sin(q_3)*np.cos(q_1)
        z_BS = -L_1*np.cos(q_1) - L_2*np.cos(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.sin(q_3) - L_3*np.cos(q_1)*np.cos(q_2)*np.cos(q_3)
        return [x_BS, y_BS, z_BS]

def main():
    rclpy.init(args=None)
    mal = ContactDetectionLocalization()
    rclpy.spin(mal)

    # Destroy the node explicitly
    mal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
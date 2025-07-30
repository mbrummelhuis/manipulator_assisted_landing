import rclpy
from rclpy.node import Node

import numpy as np

from px4_msgs.msg import SensorCombined

L_1 = 0.110
L_2 = 0.317
L_3 = 0.330

class ContactDetectionLocalization(Node):
    def __init__(self):
        super().__init__('contact_detection')

        self.declare_parameter('acc_threshold', 1.0)

        self.subscriber_accel = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_callback, 10)

        self.previous_accelero_magnitude = 0.0
        self.contact_acc = False
        self.acc_threshold = self.get_parameter('acc_threshold').get_parameter_value().double_value

        self.contact_point_1 = None
        self.contact_point_2 = None

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

    def computeLineOfAction(self):
        pass
    
    def selectContactPoint(self):
        pass

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
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import JointState
from px4_msgs.msg import SensorCombined


L_1 = 0.110
L_2 = 0.317
L_3 = 0.330

class ContactDetection(Node):
    def __init__(self):
        super().__init__('contact_detection')

        self.declare_parameter('difference_threshold', 1.)
        self.contact_threshold = self.get_parameter('difference_threshold').get_parameter_value().double_value
        self.previous_current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.subscriber_joint_states = self.create_subscription(JointState, '/servo/out/state', self.joint_states_callback, 10)

        self.subscriber_accel = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_callback, 10)

        self.contact_point_1 = None
        self.contact_point_2 = None
    

    def joint_states_callback(self, msg):
        # Calculate difference
        difference_1 = np.array(self.previous_current[0:3] - msg.effort[0:3])
        difference_2 = np.array(self.previous_current[3:6] - msg.effort[3:6])

        # Take the L2 norm of the difference per arm
        norm_1 = np.linalg.norm(difference_1)
        norm_2 = np.linalg.norm(difference_2)

        self.get_logger().info(f'Difference norms: {norm_1} \t {norm_2}')

        if norm_1 > self.contact_threshold:
            ee_1 = self.position_forward_kinematics(msg.position[0], msg.position[1], msg.position[2])
            self.contact_point_1 = ee_1
        
        if norm_2 > self.contact_threshold:
            ee_2 = self.position_forward_kinematics(msg.position[3], msg.position[4], msg.position[5])
            self.contact_point_2 = ee_2

    def sensor_callback(self, msg):
        # filter linear acceleration to detect contact
        pass

    def position_forward_kinematics(self, q_1, q_2, q_3):
        x_BS = -L_2*np.sin(q_2) - L_3*np.sin(q_2)*np.cos(q_3)
        y_BS = L_1*np.sin(q_1) + L_2*np.sin(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.cos(q_2)*np.cos(q_3) + L_3*np.sin(q_3)*np.cos(q_1)
        z_BS = -L_1*np.cos(q_1) - L_2*np.cos(q_1)*np.cos(q_2) + L_3*np.sin(q_1)*np.sin(q_3) - L_3*np.cos(q_1)*np.cos(q_2)*np.cos(q_3)
        return [x_BS, y_BS, z_BS]

def main():
    rclpy.init(args=None)
    mal = ContactDetection()
    rclpy.spin(mal)

    # Destroy the node explicitly
    mal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
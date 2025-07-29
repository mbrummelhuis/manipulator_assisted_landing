#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from pynput import keyboard

from sensor_msgs.msg import JointState
import math

class DroneManipulatorTeleop(Node):
    def __init__(self):
        super().__init__('drone_manipulator_teleop')

        self.declare_parameter('servo_increment_deg', 10.0)
        self.declare_parameter('position_clip', 0.0)
        self.servo_control_speed = self.get_parameter('servo_increment_deg').get_parameter_value().double_value

        # Publishers
        self.servo_pub = self.create_publisher(JointState, '/servo/in/state', 10)

        # Subscribers
        self.servo_sub = self.create_subscription(JointState, '/servo/out/state', self.joint_states_callback, 10)

        self.pressed_keys = set()
        self.servos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 

        # Step sizes
        self.step_servo = math.radians(self.servo_control_speed)

        self.get_logger().info("Arm 1: R/F=q1, T/G=q2, Y/H=q3")
        self.get_logger().info("Arm 2: U/J=q1, I/K=q2, O/L=q3")
        self.get_logger().info("X=exit")

        # Start keyboard listener in a separate thread
        self.running = True
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        # Periodic publisher timer (20 Hz)
        self.create_timer(0.05, self.publish_setpoints)

    def publish_setpoints(self):
        self.processKeys()

        # Servo setpoints
        if not np.linalg.norm(self.servos)==0.0:
            self.publishManipulatorSetpoints(*self.servos)

    def on_press(self, key):
        try:
            self.pressed_keys.add(key.char)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            self.pressed_keys.discard(key.char)
        except AttributeError:
            pass                

    def joint_states_callback(self, msg):
        # If the servos have not been initialized, i.e. are still 0, set to position
        if np.linalg.norm(self.servos)<5.0: # TODO make this something more sensible
            self.servos = msg.position
            self.get_logger().info(f'Set initial servo positions as {self.servos}')

    def processKeys(self):
        """Runs in background thread, updates desired state"""      
        if 'x' in self.pressed_keys:
            self.get_logger().info("Exiting teleop...")
            self.running = False
            # TODO: Add setting servos to suitable landing state
            self.land()
            rclpy.shutdown()
            return

        if 'r' in self.pressed_keys:
            self.servos[0] -= self.step_servo
        if 'f' in self.pressed_keys:
            self.servos[0] += self.step_servo
        if 't' in self.pressed_keys:
            self.servos[1] -= self.step_servo
        if 'g' in self.pressed_keys:
            self.servos[1] += self.step_servo
        if 'y' in self.pressed_keys:
            self.servos[2] -= self.step_servo
        if 'h' in self.pressed_keys:
            self.servos[2] += self.step_servo

        if 'u' in self.pressed_keys:
            self.servos[3] -= self.step_servo
        if 'j' in self.pressed_keys:
            self.servos[3] += self.step_servo
        if 'i' in self.pressed_keys:
            self.servos[4] -= self.step_servo
        if 'k' in self.pressed_keys:
            self.servos[4] += self.step_servo
        if 'o' in self.pressed_keys:
            self.servos[5] -= self.step_servo
        if 'l' in self.pressed_keys:
            self.servos[5] += self.step_servo

    def publishManipulatorSetpoints(self, q1_1, q2_1, q3_1, q1_2, q2_2, q3_2):
        msg = JointState()
        msg.position = [q1_1, q2_1, q3_1, q1_2, q2_2, q3_2]
        msg.velocity = [0., 0., 0., 0., 0., 0.]
        msg.name = ['q1_1', 'q2_1', 'q3_1', 'q1_2', 'q2_2', 'q3_2']
        msg.header.stamp = self.get_clock().now().to_msg()
        self.servo_pub.publish(msg)

def main():
    rclpy.init()
    node = DroneManipulatorTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

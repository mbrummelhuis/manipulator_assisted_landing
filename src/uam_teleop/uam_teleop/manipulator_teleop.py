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

        self.declare_parameter('servo_increment_deg', 5.0)
        self.servo_control_speed = self.get_parameter('servo_increment_deg').get_parameter_value().double_value

        # Publishers
        self.servo_pub = self.create_publisher(JointState, '/servo/in/state', 10)

        # Subscribers
        self.servo_sub = self.create_subscription(JointState, '/servo/out/state', self.joint_states_callback, 10)

        self.pressed_keys = set()
        self.servos = None

        # Map
        self.servo_keys = [
            ('r', 'f'),
            ('t', 'g'),
            ('y', 'h'),
            ('u', 'j'),
            ('i', 'k'),
            ('o', 'l'),
        ]

        # Step sizes
        self.step_servo = math.radians(self.servo_control_speed)

        self.get_logger().info("Welcome to servo keyboard teleop!")
        self.get_logger().info("Arm 1: R/F=q1, T/G=q2, Y/H=q3")
        self.get_logger().info("Arm 2: U/J=q1, I/K=q2, O/L=q3")
        self.get_logger().info("X=exit")

        # Start keyboard listener in a separate thread
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        # Periodic publisher timer (20 Hz)
        self.counter = 0
        self.create_timer(0.05, self.publish_setpoints)

        self.running = True

    def publish_setpoints(self):
        # Servo setpoints
        if self.servos is not None:
            self.publishManipulatorSetpoints(self.servos)
        else:
            if self.counter%100==0:
                self.get_logger().info("Waiting for servo feedback...")
                self.counter+=1
        
        self.processKeys()

    def on_press(self, key):
        try:
            self.pressed_keys.add(key.char)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            # Handle normal character keys
            k = key.char.lower()
            for idx, (dec_key, inc_key) in enumerate(self.servo_keys):
                if self.servos is not None and idx < len(self.servos) and k in (dec_key, inc_key):
                    self.get_logger().info(
                        f"Servo {idx} setpoint = {self.servos[idx]:.2f} [rad]"
                    )
            self.pressed_keys.discard(k)

        except AttributeError:
            # Handle special keys (ESC, arrows, etc.)
            if key == keyboard.Key.esc:
                self.get_logger().info("ESC pressed → exiting teleop...")
                self.running = False

    def joint_states_callback(self, msg):
        # If the servos have not been initialized, i.e. are still 0, set to position
        if self.servos is None: # TODO make this something more sensible
            self.servos = msg.position
            self.get_logger().info(f'Set initial servo positions as {self.servos}')

    def processKeys(self):
        for idx, (dec_key, inc_key) in enumerate(self.servo_keys):
            if self.servos is not None and idx < len(self.servos):  # only if servo exists
                if dec_key in self.pressed_keys:
                    self.servos[idx] -= self.step_servo
                if inc_key in self.pressed_keys:
                    self.servos[idx] += self.step_servo

    def publishManipulatorSetpoints(self, joint_positions:list):
        msg = JointState()
        msg.position = joint_positions
        msg.velocity = [0. for  i in range(len(joint_positions))]
        msg.name = ['q'+str(i+1) for i in range(len(joint_positions))]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.servo_pub.publish(msg)

def main():
    rclpy.init()
    node = DroneManipulatorTeleop()
    while rclpy.ok() and node.running:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

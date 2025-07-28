#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from pynput import keyboard

from sensor_msgs.msg import JointState
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
import math

class DroneManipulatorTeleop(Node):
    def __init__(self):
        super().__init__('drone_manipulator_teleop')

        self.declare_parameter('drone_increment_m', 0.1)
        self.declare_parameter('servo_increment_deg', 10.0)
        self.declare_parameter('position_clip', 0.0)
        self.drone_control_speed = self.get_parameter('drone_increment_m').get_parameter_value().double_value
        self.servo_control_speed = self.get_parameter('servo_increment_deg').get_parameter_value().double_value
        self.position_clip = self.get_parameter('position_clip').get_parameter_value().double_value

        # Publishers
        self.publisher_offboard_control_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',10)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.drone_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.servo_pub = self.create_publisher(JointState, '/servo/in/state', 10)

        # Subscribers
        self.servo_sub = self.create_subscription(JointState, '/servo/out/state', self.joint_states_callback, 10)

        self.pressed_keys = set()

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0  # start at 1m altitude
        self.yaw = 0.0
        self.servos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 

        # Step sizes
        self.step_pos = self.drone_control_speed     # meters
        self.step_alt = self.drone_control_speed
        self.step_yaw = math.radians(50*self.drone_control_speed)  # 5 deg increments
        self.step_servo = math.radians(self.servo_control_speed)

        self.get_logger().info("Teleop ready: WASD=XY, c/z=altitude, Q/E=yaw, U/J=q1, I/K=q2, O/L=q3 M=arm X=exit")


        # Start keyboard listener in a separate thread
        self.running = True
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        # Periodic publisher timer (20 Hz)
        self.create_timer(0.05, self.publish_setpoints)

    def publish_setpoints(self):
        self.processKeys()
        # Drone setpoint
        self.publishOffboardPositionMode()
        self.publishTrajectoryPositionSetpoint(
            self.x,
            self.y,
            self.z,
            self.yaw
        )

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
        if np.linalg.norm(self.servos)<5.0:
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

        if 'w' in self.pressed_keys: # Forward
            self.x += self.step_pos
        if 's' in self.pressed_keys: # Backward
            self.x -= self.step_pos

        if 'a' in self.pressed_keys: # Left
            self.y -= self.step_pos
        if 'd' in self.pressed_keys: # Right
            self.y += self.step_pos

        if 'z' in self.pressed_keys: # Lower
            self.z += self.step_alt
        if 'c' in self.pressed_keys: # Climb
            self.z -= self.step_alt

        if 'q' in self.pressed_keys: # Yaw left
            self.yaw += self.step_yaw
        if 'e' in self.pressed_keys: # Yaw right
            self.yaw -= self.step_yaw

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

        if 'm' in self.pressed_keys:
            self.armVehicle()

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    
    def armVehicle(self):
        """Send an offboard and arm command to the vehicle."""
        # Offboard command
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

        # Arm command
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

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
            x_clipped = np.clip(x, -self.position_clip, self.position_clip)
            y_clipped = np.clip(y, -self.position_clip, self.position_clip)
            z_clipped = np.clip(z, -self.position_clip, 0.0) # Negative up
        else:
            x_clipped = x
            y_clipped = y
            z_clipped = z
        msg = TrajectorySetpoint()
        msg.position[0] = x_clipped
        msg.position[1] = y_clipped
        msg.position[2] = z_clipped
        msg.yaw = yaw
        msg.yawspeed=yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.drone_pub.publish(msg)

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

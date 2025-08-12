import math
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

L1 = 0.110
L2 = 0.317
L3 = 0.330

class ManipulatorController(Node):
    def __init__(self):
        super().__init__('manipulator_controller')

        # Servo interface
        self.subscriber_servo = self.create_subscription(JointState, '/servo/out/state', self.callback_servo, 10)
        self.publisher_servo = self.create_publisher(JointState, '/servo/in/state', 10)

        # Controller interface
        self.subscriber_ee_position = self.create_subscription(TwistStamped, '/manipulator/in/position', self.callback_position, 10)
        self.subscriber_ee_velocity = self.create_subscription(TwistStamped, '/manipulator/in/velocity', self.callback_velocity, 10)

        # Data
        self.servo_state = None

    def callback_servo(self, msg):
        """
        Update the servo data
        """
        self.servo_state = msg

    def callback_position(self, msg):
        solutions_1 = self.position_inverse_kinematics([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        solutions_2 = self.position_inverse_kinematics([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])

        # If there are solutions and the servo_state is set
        if solutions_1 and solutions_2 and self.servo_state:
            lowest_diff = np.inf
            for solution1 in solutions_1:
                diff = np.linalg.norm(np.array(solution1) - np.array(self.servo_state.position[0:3]))
                if diff < lowest_diff:
                    lowest_diff = diff
                    best_solution1 = solution1
            
            lowest_diff = np.inf
            for solution2 in solutions_2:
                diff = np.linalg.norm(np.array(solution2) - np.array(self.servo_state.position[3:6]))
                if diff < lowest_diff:
                    lowest_diff = diff
                    best_solution2 = solution2

            self.get_logger().info("--- ARM 1 ---")
            self.get_logger().info(f"Current positions: {self.servo_state.position[0]:.2f} \t {self.servo_state.position[1]:.2f} \t {self.servo_state.position[2]:.2f}")
            self.get_logger().info(f"Commanded positions: {best_solution1[0]:.2f} \t {best_solution1[1]:.2f} \t {best_solution1[2]:.2f}")

            self.get_logger().info("--- ARM 2 ---")
            self.get_logger().info(f"Current positions: {self.servo_state.position[3]:.2f} \t {self.servo_state.position[4]:.2f} \t {self.servo_state.position[5]:.2f}")
            self.get_logger().info(f"Commanded positions: {best_solution2[0]:.2f} \t {best_solution2[1]:.2f} \t {best_solution2[2]:.2f}")
            self.publish_servo_positions(best_solution1 + best_solution2)
        
        elif not solutions_1 or not solutions_2 and self.servo_state:
            return

        elif not self.servo_state:
            self.get_logger().error(f'Servo state not set!')
            return

    def callback_velocity(self, msg):
        self.get_logger().info("--- ARM 1 ---")
        solution1 = self.velocity_inverse_kinematics([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], arm=1)
        self.get_logger().info("--- ARM 2 ---")
        solution2 = self.velocity_inverse_kinematics([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z], arm=2)

        if solution1 and solution2 and self.servo_state:
            self.publish_servo_velocities(solution1 + solution2)
            self.get_logger().info(f'Velocity solutions: {solution1[0]:.2f}, {solution1[1]:.2f}, {solution1[2]:.2f} \t {solution2[0]:.2f}, {solution2[1]:.2f}, {solution2[2]:.2f}')
        elif not self.servo_state:
            self.get_logger().error(f'Servo state not set!')
            return

    def position_inverse_kinematics(self, target_position:list, tol = 1e-9) -> list:
        X = target_position[0]
        Y = target_position[1]
        Z = target_position[2]
        # Step 1: compute D
        S = X**2 + Y**2 + Z**2
        D = S - (L1**2 + L2**2 + L3**2)
        
        # Step 2: quadratic coefficients for u = cos(q3)
        a = 4 * L3**2 * (L2**2 - L1**2)
        b = -4 * L2 * L3 * (D + 2 * L1**2)
        c = D**2 - 4 * L1**2 * L2**2 + 4 * L1**2 * X**2

        solutions = []

        # Step 3: solve quadratic
        if abs(a) < tol:  # degenerate to linear
            if abs(b) < tol:
                return []  # no equation to solve
            u_roots = [-c / b]
        else:
            disc = b**2 - 4*a*c
            if disc < -tol:
                return []  # no real solutions
            elif abs(disc) < tol:
                u_roots = [-b / (2*a)]
            else:
                sqrt_disc = math.sqrt(max(disc, 0.0))
                u_roots = [(-b + sqrt_disc) / (2*a),
                        (-b - sqrt_disc) / (2*a)]

        # Step 4: loop over cos(q3) solutions
        for u in u_roots:
            if abs(u) > 1 + 1e-9:
                continue  # invalid
            u = max(-1, min(1, u))  # clamp

            for s3_sign in [1, -1]:
                s3 = s3_sign * math.sqrt(max(0.0, 1 - u**2))
                C = L2 + L3 * u

                if abs(C) < tol:
                    continue  # singular, skip or handle separately

                # q2 from atan2 form
                num = -X
                den = (D - 2 * L2 * L3 * u) / (2 * L1)
                q2 = math.atan2(num, den)

                # compute A, B
                A = L1 + C * math.cos(q2)
                B = L3 * s3

                # q1
                q1 = math.atan2(Y, -Z) - math.atan2(B, A)

                # q3
                q3 = math.atan2(s3, u)
                solutions.append((q1, q2, q3))

        return solutions
    
    def velocity_inverse_kinematics(self, target_velocity:list, arm=1) -> list:
        if arm == 1:
            q1 = self.servo_state.position[0]
            q2 = self.servo_state.position[1]
            q3 = self.servo_state.position[2]
        elif arm == 2:
            q1 = self.servo_state.position[3]
            q2 = self.servo_state.position[4]
            q3 = self.servo_state.position[5]            

        self.get_logger().info(f'Current states: {q1:.2f} rad \t {q2:.2f} rad \t {q3:.2f} rad ')

         # Evaluate jacobian
        jacobian = np.empty((3,3))
        jacobian[0,0]=0.0
        jacobian[0,1]=-L2*math.cos(q2) - L3*math.cos(q2)*math.cos(q3)
        jacobian[0,2]=L3*math.sin(q2)*math.sin(q3)
        jacobian[1,0]=L1*math.cos(q1) + L2*math.cos(q1)*math.cos(q2) - L3*math.sin(q1)*math.sin(q3) + L3*math.cos(q1)*math.cos(q2)*math.cos(q3)
        jacobian[1,1]=-L2*math.sin(q1)*math.sin(q2) - L3*math.sin(q1)*math.sin(q2)*math.cos(q3)
        jacobian[1,2]=-L3*math.sin(q1)*math.sin(q3)*math.cos(q2) + L3*math.cos(q1)*math.cos(q3)
        jacobian[2,0]=L1*math.sin(q1) + L2*math.sin(q1)*math.cos(q2) + L3*math.sin(q1)*math.cos(q2)*math.cos(q3) + L3*math.sin(q3)*math.cos(q1)
        jacobian[2,1]=L2*math.sin(q2)*math.cos(q1) + L3*math.sin(q2)*math.cos(q1)*math.cos(q3)
        jacobian[2,2]=L3*math.sin(q1)*math.cos(q3) + L3*math.sin(q3)*math.cos(q1)*math.cos(q2)
        
        # Evaluate jacobian
        jacobian_pinv = np.linalg.pinv(jacobian)
        joint_velocities = jacobian_pinv @ np.array(target_velocity)

        self.get_logger().info(f'Velocities: {joint_velocities[0]:.2f} rad \t {joint_velocities[1]:.2f} rad \t {joint_velocities[2]:.2f} rad ')

        EE_velocity = jacobian @ joint_velocities

        self.get_logger().info(f'EE velocity: {EE_velocity[0]:.2f}, {EE_velocity[1]:.2f}, {EE_velocity[2]:.2f}')

        return joint_velocities.tolist()

    
    def publish_servo_positions(self, positions):
        clipped_positions = np.empty(len(positions))
        clipped_positions[0] = np.clip(positions[0], -np.pi, np.pi)
        clipped_positions[1] = np.clip(positions[1], -np.pi/8., np.pi/8.)
        clipped_positions[2] = np.clip(positions[2], -1.85, 1.85)
        clipped_positions[3] = np.clip(positions[3], -np.pi, np.pi)
        clipped_positions[4] = np.clip(positions[4], -np.pi/8., np.pi/8.)
        clipped_positions[5] = np.clip(positions[5], -1.85, 1.85)

        msg = JointState()
        msg.name = ['q'+str(i) for i in range(len(clipped_positions))]
        msg.position = clipped_positions
        msg.velocity = [0.0 for i in range(len(clipped_positions))]
        msg.effort = [0.0 for i in range(len(clipped_positions))]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_servo.publish(msg)
    
    def publish_servo_velocities(self, velocities):
        msg = JointState()
        msg.name = ['q'+str(i) for i in range(len(velocities))]
        msg.position = [0.0 for i in range(len(velocities))]
        msg.velocity = velocities
        msg.effort = [0.0 for i in range(len(velocities))]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_servo.publish(msg)

def main():
    rclpy.init(args=None)
    manipulator_controller = ManipulatorController()
    rclpy.spin(manipulator_controller)

    # Destroy the node explicitly
    manipulator_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
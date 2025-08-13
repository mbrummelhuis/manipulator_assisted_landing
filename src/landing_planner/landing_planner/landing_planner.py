import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import PointStamped, Vector3Stamped, TwistStamped
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint


class LandingPlanner(Node):
    def __init__(self):
        super().__init__('landing_planner')

        self.declare_parameter('dimension', 2) # Dimension of the problem, 2D or 3D (2 or 3)
        self.declare_parameter('landing_offset', 0.5)
        self.dimension = self.get_parameter('dimension').get_parameter_value().integer_value

        # Subscriber
        self.subscription_contact_point = self.create_subscription(PointStamped, '/contact/out/contact_point', self.contact_point_callback, 10)
        self.subscription_local_position = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, 10)

        # Publisher
        self.publisher_plane_centroid = self.create_publisher(PointStamped, '/landing/out/centroid', 10) # For logging
        self.publisher_plane_normal = self.create_publisher(Vector3Stamped, '/landing/out/normal', 10) # For logging

        self.publisher_landing_start = self.create_publisher(TrajectorySetpoint, '/landing/out/start_location', 10)
        self.publisher_manipulator_positions = self.create_publisher(TwistStamped, '/landing/out/manipulator', 10)
        
        # Data
        self.landing_offset = self.get_parameter('landing_offset').get_parameter_value().double_value
        self.contact_points = []
        self.centroid_world = None
        self.centroid_body = None
        self.normal_world = None
        self.normal_body = None
        self.x_axis = np.array([1., 0., 0.])
        self.body_position = None
        self.body_heading = None
        self.landing = False
        self.horizontal_leg_man_distance = 0.27
        self.x_manipulator_distance = 0.08
        self.y_manipulator_distance = 0.4
        self.leg_z = 0.09


    def plan_landing(self):
        # Landing start point above the centroid of the plane
        landing_point = self.centroid_world + np.array([0., 0., -self.landing_offset])
        heading = self.plane_heading_rad()
        self.publish_landing_point(landing_point, heading)

        # Calculate end-effector desired locations in body frame
        manipulator_body_z = self.leg_z + self.horizontal_leg_man_distance*np.tan(self.calculate_surface_incline(self.normal_world))
        arm1_ee_position_body = np.array([self.x_manipulator_distance, self.y_manipulator_distance, manipulator_body_z])
        arm2_ee_position_body = np.array([self.x_manipulator_distance, -self.y_manipulator_distance, manipulator_body_z])

        self.get_logger().info(f'Manipulator 1 body targets: {arm1_ee_position_body[0]:.2f}, {arm1_ee_position_body[1]:.2f}, {arm1_ee_position_body[2]:.2f}')
        self.get_logger().info(f'Manipulator 2 body targets: {arm2_ee_position_body[0]:.2f}, {arm2_ee_position_body[1]:.2f}, {arm2_ee_position_body[2]:.2f}')
        
        self.publish_desired_manipulator_positions(arm1_ee_position_body, arm2_ee_position_body)

    
    def contact_point_callback(self, msg):
        """
        Add contact points to the database and if enough have been gathered, determine the plane
        """
        new_point = np.array([msg.x, msg.y, msg.z])
        self.contact_points.append(new_point)
        # Once enough points have been gathered, calculate the plane
        if len(self.contact_points) == 2 and self.dimension == 2: # 2D case
            self.centroid_world, self.normal_world, self.centroid_body, self.normal_body = self.determine_plane_2D()
            self.landing = True
            
        elif len(self.contact_points) == 3 and self.dimension == 3: # 3D case
            self.centroid_world, self.normal_world, self.centroid_body, self.normal_body = self.determine_plane_3D()
            self.landing = True
        return

    def determine_plane_2D(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Fit a plane through two contact points,
        constrained to be parallel to the drone's body x-axis.
        
        Returns:
            centroid (np.ndarray): point on the plane in world frame
            normal (np.ndarray): unit normal vector of the plane in world frame
        """
        line_vec_body = self.contact_points[1] - self.contact_points[0]
        centroid_body = (self.contact_points[1] + self.contact_points[0]) / 2

        # Cross product gives the plane normal
        normal_body = np.cross(line_vec_body, self.x_axis)

        if self.body_position is not None:
            centroid_world = centroid_body + self.body_position
            normal_world = self.rotate_body_to_world(normal_body)
        else:
            centroid_world = centroid_body
            normal_world = normal_body
            self.get_logger().warn(f'No body position to transform centroid and normal from body to world frame!')

        # Handle degenerate case: if line_vec ∥ x_axis, cross product is zero
        if np.linalg.norm(normal) < 1e-6:
            raise ValueError("Line vector and x-axis are parallel, cannot define a unique plane.")

        normal /= np.linalg.norm(normal)
        self.publish_centroid(centroid_world)
        self.publish_normal(normal_world)
        return centroid_world, normal_world, centroid_body, normal_body
        
    def determine_plane_3D(self) -> tuple[np.array, np.array]:
        '''
        Determine the best-fitting plane between all the contact points
        '''
        # If not enough points present, return
        if len(self.contact_points) < self.dimension:
            return None

        # Center the points
        points = np.array(self.contact_points)
        centroid_body = np.mean(points, axis=0)
        centered = points - centroid_body
        # Singular Value Decomposition
        _, _, vh = np.linalg.svd(centered)
        normal_body = vh[-1]  # last row of vh corresponds to smallest singular value

        if self.body_position is not None:
            centroid_world = centroid_body + self.body_position
            normal_world = self.rotate_body_to_world(normal_body)
        else:
            centroid_world = centroid_body
            normal_world = normal_body
            self.get_logger().warn(f'No body position to translate centroid from body to world frame!')

        self.get_logger().info(f'Calculated plane centroid (Fb) {centroid_body} and normal vector (Fb) {normal_body}')
        self.publish_centroid(centroid_world)
        self.publish_normal(normal_world)
        return centroid_world, normal_world, centroid_body, normal_body  # plane passes through centroid with this normal

    # Helper functions
    def plane_heading_rad(self):
        """
        Compute the heading of a plane given its normal vector.
        Heading is the angle (in radians) between the plane's projection 
        onto the horizontal plane and North (y-axis), in [0, 2pi).

        
        Returns:
            heading: float, radians in [0, 2pi)
        """
        nx, ny, _ = self.normal_world
        if np.isclose(nx, 0) and np.isclose(ny, 0):
            return 0.0  # horizontal plane, heading undefined, assume 0
        heading = np.arctan2(nx, ny)  # angle from North (y-axis)
        return heading % (2*np.pi)

    def calculate_surface_incline(normal):
        z_axis = np.array([0,0,1]) # Up to comply with intuition
        return np.arccos(np.abs(np.dot(normal, z_axis)) / np.linalg.norm(normal))

    
    def publish_normal(self, vector:np.array):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = vector[0]
        msg.vector.y = vector[1]
        msg.vector.z = vector[2]
        self.publisher_plane_normal.publish(msg)

    def publish_centroid(self, point:np.array):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = point[0]
        msg.point.y = point[1]
        msg.point.z = point[2]
        self.publisher_plane_centroid.publish(msg)

    def publish_landing_point(self, point:np.array, heading:float):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg.position[0] = point[0]
        msg.position[1] = point[1]
        msg.position[2] = point[2]
        msg.yaw = heading
        self.publisher_landing_start.publish(msg)

    def publish_desired_manipulator_positions(self, arm1_ee_location:np.array, arm2_ee_location:np.array):
        msg = TwistStamped()
        msg.twist.linear.x = arm1_ee_location[0]
        msg.twist.linear.y = arm1_ee_location[1]
        msg.twist.linear.z = arm1_ee_location[2]
        msg.twist.angular.x = arm2_ee_location[0]
        msg.twist.angular.y = arm2_ee_location[1]
        msg.twist.angular.z = arm2_ee_location[2]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_manipulator_positions.publish(msg)

        
    def local_position_callback(self, msg):
        self.x_axis[0] = np.cos(msg.heading)
        self.x_axis[1] = np.sin(msg.heading)
        self.x_axis[2] = 0.

        self.body_heading = msg.heading

        self.body_position[0] = msg.x
        self.body_position[1] = msg.y
        self.body_position[2] = msg.z

        # TODO: Also consider pitch and yaw from the orientation quaternion

    def rotate_body_to_world(self, vector:np.array):
        rotmat = np.array([[np.cos(self.body_heading), -np.sin(self.body_heading), 0.0],
                           [np.sin(self.body_heading), np.cos(self.body_heading), 0.0],
                           [0.0, 0.0, 1.0]])
        return rotmat @ vector


def main():
    rclpy.init(args=None)
    landingplanner = LandingPlanner()
    rclpy.spin(landingplanner)

    # Destroy the node explicitly
    landingplanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
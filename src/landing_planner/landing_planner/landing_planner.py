import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import Point, Vector3
from px4_msgs.msg import VehicleLocalPosition


class LandingPlanner(Node):
    def __init__(self):
        super().__init__('landing_planner')

        self.declare_parameter('dimension', 2) # Dimension of the problem, 2D or 3D (2 or 3)
        self.declare_parameter('frequency', 10.)
        self.dimension = self.get_parameter('dimension').get_parameter_value().integer_value

        # Subscriber
        self.subscription_contact_point = self.create_subscription(Point, '/landing/in/contact_point', self.contact_point_callback, 10)
        self.subscription_local_position = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, 10)

        # Publisher
        self.publisher_plane_centroid = self.create_publisher(Point, '/landing/out/centroid', 10) # For logging
        self.publisher_plane_normal = self.create_publisher(Vector3, '/landing/out/normal', 10) # For logging
        
        # Data
        self.contact_points = []
        self.centroid = None
        self.normal = None
        self.x_axis = np.array([1., 0., 0.])

        # Timer -- always last
        self.counter = 0
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    def contact_point_callback(self, msg):
        new_point = np.array([msg.x, msg.y, msg.z])
        self.contact_points.append(new_point)
        # Once enough points have been gathered, calculate the plane
        if len(self.contact_points) == 2 and self.dimension == 2: # 2D case
            self.centroid, self.normal = self.determine_plane_2D()
        elif len(self.contact_points) > 2 and self.dimension == 3: # 3D case
            self.centroid, self.normal = self.determine_plane_3D()
        return

    def determine_plane_2D(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Fit a plane through two contact points,
        constrained to be parallel to the drone's body x-axis.
        
        Returns:
            centroid (np.ndarray): point on the plane
            normal (np.ndarray): unit normal vector of the plane
        """
        line_vec = self.contact_points[1] - self.contact_points[0]
        centroid = (self.contact_points[1] + self.contact_points[0]) / 2

        # Cross product gives the plane normal
        normal = np.cross(line_vec, self.x_axis)

        # Handle degenerate case: if line_vec ∥ x_axis, cross product is zero
        if np.linalg.norm(normal) < 1e-6:
            raise ValueError("Line vector and x-axis are parallel, cannot define a unique plane.")

        normal /= np.linalg.norm(normal)
        self.publish_centroid(centroid)
        self.publish_normal(normal)
        return centroid, normal
        
    def determine_plane_3D(self) -> tuple[np.array, np.array]:
        '''
        Determine the best-fitting plane between all the contact points
        '''
        # If not enough points present, return
        if len(self.contact_points) < self.dimension:
            return None

        # Center the points
        points = np.array(self.contact_points)
        centroid = np.mean(points, axis=0)
        centered = points - centroid

        # Singular Value Decomposition
        _, _, vh = np.linalg.svd(centered)
        normal = vh[-1]  # last row of vh corresponds to smallest singular value

        self.get_logger().info(f'Calculated plane centroid {centroid} and normal vector {normal}')
        self.publish_centroid(centroid)
        self.publish_normal(normal)
        return centroid, normal  # plane passes through centroid with this normal

    def publish_normal(self, vector:np.array):
        msg = Vector3()
        msg.x = vector[0]
        msg.y = vector[1]
        msg.z = vector[2]
        self.publisher_plane_normal.publish(msg)

    def publish_centroid(self, point:np.array):
        msg = Point()
        msg.x = point[0]
        msg.y = point[1]
        msg.z = point[2]
        self.publisher_plane_centroid.publish(msg)

    def local_position_callback(self, msg):
        self.x_axis[0] = np.cos(msg.heading)
        self.x_axis[1] = np.sin(msg.heading)
        self.x_axis[2] = 0.
        # TODO: Also consider pitch and yaw from the orientation quaternion


def main():
    rclpy.init(args=None)
    landingplanner = LandingPlanner()
    rclpy.spin(landingplanner)

    # Destroy the node explicitly
    landingplanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
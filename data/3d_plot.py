
from __future__ import annotations

from pathlib import Path

from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from rosbags.rosbag2 import Reader

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Patch
import numpy as np

"""
Example file on how to register px4_msgs types with rosbags

"""
# ----------------------------------------------------------------------------------------------
# READING DATA FROM ROSBAG
# ----------------------------------------------------------------------------------------------
px4_msgs_path = '/home/martijn/manipulator_assisted_landing/src/px4_msgs/msg/'
rosbag_path = '/home/martijn/manipulator_assisted_landing/data/rosbags/ros2bag_30deg_success1'

def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)


typestore = get_typestore(Stores.ROS2_JAZZY)
add_types = {}

for pathstr in [px4_msgs_path+'VehicleLocalPosition.msg', px4_msgs_path+'TrajectorySetpoint.msg',]:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

typestore.register(add_types)

VehicleLocalPosition = typestore.types['px4_msgs/msg/VehicleLocalPosition']

vehicle_x = []
vehicle_y = []
vehicle_z = []
contact_x_body = []
contact_y_body = []
contact_z_body = []
contact_x_world = []
contact_y_world = []
contact_z_world = []
vehicle_time = []
contact_points_time = []
initial_timestamp = None
normal = []
centroid = []
heading = []


# Create reader instance and open for reading.
with Reader(rosbag_path) as reader:
    # Topic and msgtype information is available on .connections list.
    # for connection in reader.connections:
    #     print(connection.topic, connection.msgtype)

    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/fmu/out/vehicle_local_position':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            if len(vehicle_time) == 0: # Save the first timestamp for subtraction
                initial_timestamp = msg.timestamp
            vehicle_x.append(msg.x)
            vehicle_y.append(msg.y)
            vehicle_z.append(msg.z)
            vehicle_time.append(msg.timestamp)
        if connection.topic == '/contact/out/contact_point_coords':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            contact_x_body.append(msg.point.x)
            contact_y_body.append(msg.point.y)
            contact_z_body.append(msg.point.z)
            contact_points_time.append(msg.header.stamp)
            print("Contact point (body frame): ", msg.point.x, msg.point.y, msg.point.z)
            # Select the only the first 2 contact points
            # Transform to world frame
        if connection.topic == '/landing/out/normal':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            normal = [msg.vector.x, msg.vector.y, msg.vector.z]
            print("Surface normal: ", normal)
        if connection.topic == '/landing/out/centroid':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            centroid = [msg.point.x, msg.point.y, msg.point.z]
            print("Centroid (world frame)", centroid)

        if connection.topic == '/landing/out/start_location':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            heading.append(msg.yaw)
            print("Heading (world frame): ", heading)

# Find vehicle xyz with the closest timestamp to the contact time
for i, contact_timestamp in enumerate(contact_points_time):
    point_time_us = contact_timestamp.sec*1e6 + contact_timestamp.nanosec*1e-3 # In microsecs to correspond to px4
    vehicle_time_array = np.array(vehicle_time)
    idx = np.argmin(np.abs(vehicle_time_array - point_time_us))
    x = vehicle_x[idx]
    y = vehicle_y[idx]
    z = vehicle_z[idx]
    contact_x_world.append(contact_x_body[i] + x)
    contact_y_world.append(contact_y_body[i] + y)
    contact_z_world.append(contact_z_body[i] + z)
    print("Contact point (world frame): ", contact_x_body[i]+x, contact_y_world[i], contact_z_world[i])

# Discard first contact point as it seems faulty
contact_x_world = contact_x_world[1:]
contact_y_world = contact_y_world[1:]
contact_z_world = contact_z_world[1:]

# Set start of flight as t=0
vehicle_time = [stamp - initial_timestamp for stamp in vehicle_time]

# ----------------------------------------------------------------------------------------------
# PLOTTING
# ----------------------------------------------------------------------------------------------

# --- Create figure and 3D axis ---
fig = plt.figure(figsize=(10,7))
ax = fig.add_subplot(111, projection='3d')

print("Vehicle XYZ lengths: ", len(vehicle_x), len(vehicle_y), len(vehicle_z))

# --- Plot trajectory line ---
ax.plot(vehicle_x, vehicle_y, vehicle_z, label='Trajectory', color='blue', linewidth=2)

# --- Scatter a few points along the trajectory ---
# scatter_indices = [0, 2, 4]  # example indices to highlight
# ax.scatter(np.array(x)[scatter_indices],
#            np.array(y)[scatter_indices],
#            np.array(z)[scatter_indices],
#            color='red', s=50, label='Sample Points')

# --- Add transparent planes ---
# Plane 1: Perceived plane
# Three points defining the plane
P11 = np.array([contact_x_world[0], contact_y_world[0], contact_z_world[0]])
P21 = np.array([contact_x_world[1], contact_y_world[1], contact_z_world[1]])
P31 = P21 + np.array([np.sin(heading[0]), np.cos(heading[0]), 0])

print("Perceived plane points: ", P11, P21, P31)

# Compute vectors
v11 = P21 - P11
v21 = P31 - P11

# Normal vector
n = np.cross(v11, v21)
a, b, c = n

# Generate XY grid covering the plane
xx, yy = np.meshgrid(np.linspace(-0.75, 0.75, 10), np.linspace(-1.8, -1., 10))

# Solve for Z
zz1 = (-a*(xx - P11[0]) - b*(yy - P11[1])) / c + P11[2]

ax.plot_surface(xx, yy, zz1, color='green', alpha=0.3)
legend_proxy_detected_surface = Patch(facecolor="green", edgecolor="green", alpha=0.3, label="Detected Surface")

# Plane 2: Actual plane
# Three points defining the plane
actual_incline = np.deg2rad(30.3) # Actual incline in radians
P12 = np.array(centroid)
P22 = np.array(centroid) + np.array([0, np.cos(actual_incline), np.sin(actual_incline)])
P32 = P12 + np.array([1, 0, 0])

print("Real plane points: ", P12, P22, P32)

# Compute vectors
v12 = P22 - P12
v22 = P32 - P12

# Normal vector
n = np.cross(v12, v22)
a, b, c = n

# Solve for Z
zz2 = (-a*(xx - P12[0]) - b*(yy - P12[1])) / c + P12[2]

ax.plot_surface(xx, yy, zz2, color='orange', alpha=0.3)
legend_proxy_detected_surface = Patch(facecolor="orange", edgecolor="orange", alpha=0.3, label="Reference Surface")

# Optional: set equal aspect ratio for better visualization
# Compute limits
max_range = max(np.ptp(vehicle_x), np.ptp(vehicle_y), np.ptp(vehicle_z))

mid_x = 0.5 * (np.max(vehicle_x) + np.min(vehicle_x))
mid_y = 0.5 * (np.max(vehicle_y) + np.min(vehicle_y))
mid_z = 0.5 * (np.max(vehicle_z) + np.min(vehicle_z))

# Set equal limits for all axes
ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

# Ensure cubic aspect
ax.set_box_aspect([1, 1, 1])

# --- Labels and title ---
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.invert_zaxis()
ax.legend()

plt.show()
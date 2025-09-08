
from __future__ import annotations

from pathlib import Path

from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from rosbags.rosbag2 import Reader

import matplotlib.pyplot as plt
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
            print("Estimated incline angle: ", 90.+np.rad2deg(np.arctan(normal[2]/normal[1])))
        if connection.topic == '/landing/out/centroid':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            centroid = [msg.point.x, msg.point.y, msg.point.z]
            print("Centroid (world frame)", centroid)
            # Correct the contact body points to world frame through centroid
            dx_body = contact_x_body[1] - contact_x_body[2]
            dy_body = contact_y_body[1] - contact_y_body[2]
            dz_body = contact_z_body[1] - contact_z_body[2]
            d_body = np.array([dx_body, dy_body, dz_body])
            d_body_half = d_body/2.
            contact_x_world.append(centroid[0] + d_body_half[0])
            contact_y_world.append(centroid[1] + d_body_half[1])
            contact_z_world.append(centroid[2] + d_body_half[2])

            contact_x_world.append(centroid[0] - d_body_half[0])
            contact_y_world.append(centroid[1] - d_body_half[1])
            contact_z_world.append(centroid[2] - d_body_half[2])
        if connection.topic == '/landing/out/start_location':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            heading.append(msg.yaw)
            print("Heading (world frame): ", heading)

# Find vehicle xyz with the closest timestamp to the contact time
# for i, contact_timestamp in enumerate(contact_points_time):
#     point_time_us = contact_timestamp.sec*1e6 + contact_timestamp.nanosec*1e-3 # In microsecs to correspond to px4
#     vehicle_time_array = np.array(vehicle_time)
#     idx = np.argmin(np.abs(vehicle_time_array - point_time_us))
#     x = vehicle_x[idx]
#     y = vehicle_y[idx]
#     z = vehicle_z[idx]
#     contact_x_world.append(contact_x_body[i] + x)
#     contact_y_world.append(contact_y_body[i] + y)
#     contact_z_world.append(contact_z_body[i] + z)
#     print("Contact point (world frame): ", contact_x_body[i]+x, contact_y_world[i], contact_z_world[i])

# Discard first contact point as it seems faulty
# contact_x_world = contact_x_world[1:]
# contact_y_world = contact_y_world[1:]
# contact_z_world = contact_z_world[1:]

# Set start of flight as t=0
vehicle_time = [stamp - initial_timestamp for stamp in vehicle_time]

# ----------------------------------------------------------------------------------------------
# PLOTTING
# ----------------------------------------------------------------------------------------------
line_length = 2.2
fontsize = 25

# ---- Line A: Actual plane ----
cross_point_x = (contact_y_world[0] + contact_y_world[1])/2.
cross_point_y = (contact_z_world[0] + contact_z_world[1])/2.
angle_deg = 30.5
angle_rad = np.deg2rad(angle_deg)
x_start = cross_point_x-cross_point_y/np.tan(angle_rad)
y_start = 0.0
dirvec = np.array([-np.cos(angle_rad), -np.sin(angle_rad)])
x_end = dirvec[0]*line_length+x_start
y_end = dirvec[1]*line_length+y_start

x_lineA = [x_start, x_end]  # start at y=0
y_lineA = [y_start, y_end]  # use centroid z as reference

# ---- Line B: Determined plane ----
dx_dir = contact_y_world[1] - contact_y_world[0]
dy_dir = contact_z_world[1] - contact_z_world[0]
angle_rad = np.arctan2(dy_dir, dx_dir)
normB = np.hypot(dx_dir, dy_dir)
x_start = contact_y_world[1]-contact_z_world[1]/np.tan(angle_rad)
y_start = 0.0
dirvec = np.array([-np.cos(angle_rad), -np.sin(angle_rad)])
x_end = dirvec[0]*line_length+x_start
y_end = dirvec[1]*line_length+y_start

y_lineB = [x_start, x_end]
z_lineB = [y_start, y_end]

# ---- Plotting ----
fig, ax = plt.subplots(figsize=(7, 7))

# Trajectory
ax.plot(vehicle_y, vehicle_z, linewidth = 3, label="UAM trajectory", color="blue")

# Points
ax.scatter(contact_y_world, contact_z_world, color="red", s=150, marker="o", label="Contact points")
ax.scatter(vehicle_y[0], vehicle_z[0], color="#66B2FF", s=150, marker="o", label="Start position")
ax.scatter(vehicle_y[-1], vehicle_z[-1], color="#000099", s=150, marker="o", label="End position")

# Normal
plt.arrow(centroid[1], centroid[2], normal[1]/10., normal[2]/10., linewidth=2.5, head_width=0.06, head_length=0.06, fc='red', ec='red')
plt.annotate(
    r"$n_s$",            # LaTeX text to place at the arrow tip
    xy=(centroid[1]+normal[1]/10., centroid[2]+normal[2]/10.),       # Arrow tip position
    xytext=(centroid[1]+normal[1]/10.-0.1, centroid[2]+normal[2]/10.-0.1),  # Arrow tail position
    fontsize=fontsize,
    color="black"
)

# Centroid
ax.scatter(centroid[1], centroid[2], color="black", s=80, marker="x", label="Centroid")
# ax.scatter(cross_point_x, cross_point_y, color="green", s=80, marker="x", label="Cross point")

# Lines
ax.plot(x_lineA, y_lineA, linestyle="--", linewidth=4, color="green", alpha=0.6, label=f"Reference plane")
ax.plot(y_lineB, z_lineB, linestyle="--", linewidth=4, color="purple", alpha=0.6, label="Determined plane")
# Thick black line as x-axis
ax.axhline(y=0, color='black', linewidth=4)

# Formatting
ax.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.7)
ax.set_xlabel("Y position [m]", fontsize=fontsize)
ax.set_ylabel("Z position [m]", fontsize=fontsize)
ax.set_xlim([-3.5, 0.22])
plt.xticks(fontsize=fontsize)
plt.yticks(fontsize=fontsize)
ax.set_aspect("equal", adjustable="box")
ax.invert_yaxis()

ax.legend(fontsize=fontsize, loc='lower left', bbox_to_anchor=(0, 0.07))  # shift slightly up
plt.show()
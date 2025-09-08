
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
torque_x = []
torque_y = []
torque_z = []
time_torque = []


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
        if connection.topic == '/contact/out/estimated_torque':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            torque_x.append(msg.vector.x)
            torque_y.append(msg.vector.y)
            torque_z.append(msg.vector.z)
            if len(time_torque) == 0:
                initial_timestamp_torque = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            time_torque.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)

# Set experiment start as t=0
time_torque = [stamp - (initial_timestamp_torque) for stamp in time_torque]

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
fontsize = 25

# ---- Plotting ----
fig, ax = plt.subplots(figsize=(20, 8))

# Trajectory
ax.plot(time_torque, torque_x, linewidth = 3, label="Roll torque", color="green")
ax.plot(time_torque, torque_y, linewidth = 3, label="Pitch torque", color="blue")
ax.plot(time_torque, torque_z, linewidth = 3, label="Yaw torque", color="red")

# Formatting
ax.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.7)
ax.set_xlabel("Time [sec]", fontsize=fontsize)
ax.set_ylabel("Estimated roll torque [Nm]", fontsize=fontsize)
plt.xticks(fontsize=fontsize)
plt.yticks(fontsize=fontsize)

ax.legend(fontsize=fontsize, loc='upper left')  # shift slightly up
plt.show()
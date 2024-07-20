# import rosbag
# import matplotlib.pyplot as plt

# bag_file = '/home/moorthi/slam_ws/src/explorer_bot/bags/2024-07-19-23-01-29.bag'

# # Variables to store data
# time_cmd = []
# cmd_vel = []
# time_dist = []
# distance = []

# # Open the bag file
# bag = rosbag.Bag(bag_file)
# start_time = None

# # Extract data from the bag file
# for topic, msg, t in bag.read_messages(topics=['/cmd_vel', '/odom']):
#     if start_time is None:
#         start_time = t.to_sec()

#     if topic == '/cmd_vel':
#         time_cmd.append(t.to_sec() - start_time)
#         cmd_vel.append(msg.linear.x)
#     elif topic == '/odom':
#         time_dist.append(t.to_sec() - start_time)
#         distance.append(msg.pose.pose.position.x)

# bag.close()

# # Plot the data
# plt.figure()

# plt.subplot(2, 1, 1)
# plt.plot(time_cmd, cmd_vel, label='Commanded Velocity')
# plt.xlabel('Time (s)')
# plt.ylabel('Velocity (m/s)')
# plt.legend()

# plt.subplot(2, 1, 2)
# plt.plot(time_dist, distance, label='Distance')
# plt.xlabel('Time (s)')
# plt.ylabel('Distance (m)')
# plt.legend()

# plt.tight_layout()
# plt.show()

import rosbag
import matplotlib.pyplot as plt
import numpy as np

bag_file = '/home/moorthi/slam_ws/src/explorer_bot/bags/2024-07-19-23-01-29.bag'

# Variables to store data
time_cmd = []
cmd_vel = []
time_dist = []
distance = []

# Open the bag file
bag = rosbag.Bag(bag_file)
start_time = None

# Extract data from the bag file
for topic, msg, t in bag.read_messages(topics=['/cmd_vel', '/odom']):
    if start_time is None:
        start_time = t.to_sec()

    if topic == '/cmd_vel':
        time_cmd.append(t.to_sec() - start_time)
        cmd_vel.append(msg.linear.x)
    elif topic == '/odom':
        time_dist.append(t.to_sec() - start_time)
        distance.append(msg.pose.pose.position.x)

bag.close()

# Convert lists to numpy arrays for easier handling
time_cmd = np.array(time_cmd)
cmd_vel = np.array(cmd_vel)
time_dist = np.array(time_dist)
distance = np.array(distance)

# Align distance data with cmd_vel data
aligned_distances = []
aligned_cmd_velocities = []

for t_cmd, v in zip(time_cmd, cmd_vel):
    # Find the closest distance timestamp
    closest_time = min(time_dist, key=lambda t_dist: abs(t_dist - t_cmd))
    closest_distance = distance[np.where(time_dist == closest_time)][0]
    aligned_distances.append(closest_distance)
    aligned_cmd_velocities.append(v)

aligned_distances = np.array(aligned_distances)
aligned_cmd_velocities = np.array(aligned_cmd_velocities)

# Plot the data
plt.figure()
plt.plot(aligned_distances, aligned_cmd_velocities, marker='o', linestyle='-', color='b')
plt.xlabel('Distance (m)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity vs Distance')
plt.grid(True)
plt.show()


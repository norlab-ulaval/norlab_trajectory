import trajectory
import numpy as np
import copy
import matplotlib.pyplot as plt
import csv

# Create an empty list to store the timestamp and transformation matrix tuples
pose_list = []
traj_estim =[]

# Open the CSV file
with open('/home/effie/norlab_trajectory_test/odom_data.csv', newline='') as csvfile:
    # Create a CSV reader object
    reader = csv.reader(csvfile, delimiter=',')

    # Skip the header row
    next(reader)

    # Loop through the remaining rows
    for row in reader:
        # Extract the values
        timestamp, x, y, z, qx, qy, qz, qw = row

        # Convert strings to floats
        x, y, z, qx, qy, qz, qw = map(float, [x, y, z, qx, qy, qz, qw])

        # Compute rotation matrix from quaternion
        T = np.array([
            [1 - 2 * qy ** 2 - 2 * qz ** 2, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw, x],
            [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx ** 2 - 2 * qz ** 2, 2 * qy * qz - 2 * qx * qw, y],
            [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx ** 2 - 2 * qy ** 2, z],
            [0, 0, 0, 1]])

        # Append the timestamp and matrix as a tuple to the list
        pose_list.append((timestamp, copy.deepcopy(T)))

# Create the trajectory from the poses
traj = trajectory.Trajectory(pose_list)

# Create a trajectory interpolation
time_stamps = np.linspace(pose_list[0][0],pose_list[-1][0],100)
for time in time_stamps :
    traj_estim.append(traj.getPose(time))

# Variables x and y of the trajectory
values_x = [pose[1][0][3] for pose in pose_list]
values_y = [pose[1][1][3] for pose in pose_list]
values_x_estim = [pose[0][3] for pose in traj_estim]
values_y_estim = [pose[1][3] for pose in traj_estim]

# Create the graphics
plt.plot(values_x, values_y, '.', label='measured values')
plt.plot(values_x_estim, values_y_estim,'-', label = 'interpolated values')
plt.legend()
plt.show()

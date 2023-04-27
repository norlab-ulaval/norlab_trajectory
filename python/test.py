import trajectory
import numpy as np
import copy

pose = np.eye(4)
poses = []
for i in range(10):
    pose[0, 3] = float(i)
    poses.append((float(i), copy.deepcopy(pose)))
traj = trajectory.Trajectory(poses)

print("Hello World!")

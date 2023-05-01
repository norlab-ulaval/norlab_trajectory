import trajectory
import numpy as np
import copy
import matplotlib.pyplot as plt

pose = np.eye(4)
poses = []
for i in range(10):
    pose[0, 3] = float(i)
    # pose[0,3] = np.random.randint(0,10)
    pose[1,3] = np.random.randint(0,10)
    poses.append((float(i), copy.deepcopy(pose)))
traj = trajectory.Trajectory(poses)

time_stamps = np.linspace(poses[0][0],poses[-1][0],100)
traj_estim = []
for time_stamp in time_stamps :
    traj_estim.append(traj.getPose(time_stamp))

values_x = [pose[1][0][3] for pose in poses]
values_y = [pose[1][1][3] for pose in poses]

print(poses[2][1])
print(traj.getPose(2.5))

values_x_estim = [pose[0][3] for pose in traj_estim]
values_y_estim = [pose[1][3] for pose in traj_estim]

plt.plot(values_x, values_y, '.', label='measured values')
plt.plot(values_x_estim, values_y_estim,'-', label = 'interpolated values')
plt.legend()
plt.show()
#print("Hello World!")

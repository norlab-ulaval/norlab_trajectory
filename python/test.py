import trajectory
import numpy as np

pose =np.array([[0,0,0,1],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0]])

pose_list=([(0,np.array([[0,0,0,1],
                          [0,0,0,0],
                          [0,0,0,0],
                          [0,0,0,0]])),
            (1,np.array([[0,0,0,1],
                         [0,0,0,0],
                         [0,0,0,0],
                         [0,0,0,1]])),
            (2,np.array([[0,0,0,1],
                         [0,0,0,0],
                         [0,0,0,0],
                         [0,0,0,2]]))])
test = []
time = 0
for i in range(10) :
    time +=1
    pose[3][3]+= 1
    test.append((time,pose))

traj_test = trajectory.Trajectory(pose_list)

# print("hello")

# trajectory.Trajectory(poses: List[Tuple[float, numpy.ndarray[float32[4, 4]]]])


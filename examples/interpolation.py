import copy
import os
from pathlib import Path

import matplotlib.colors as colors
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import numpy as np
import pandas as pd
from matplotlib.patches import Ellipse
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from tqdm import tqdm

import norlab_trajectory

BASE_PATH = Path(__file__).absolute().parents[1] / 'examples' / 'trajectories_examples'
OUTPUT_PATH = Path(__file__).absolute().parents[1] / 'examples' / 'output'
import argparse


def main(input_file, covariance, points, quiver, output):
    """
    This function reads a csv file containing a trajectory and interpolates it with a given covariance value.
    :param input_file: Name of the input file containing the trajectory of the robot.
    :param covariance: Covariance value for interpolation.
    :param points: Number of points between two timestamps, default value is 1.
    :param quiver: Quiver plot for the normal vectors.
    """
    traj_estim = []
    df = pd.read_csv(BASE_PATH / f'{input_file}', usecols=["Timestamp", "X", "Y", "Z", "qx", "qy", "qz", "qw"])

    timestamps = df['Timestamp'].values
    poses = []

    for i in range(len(timestamps)):
        x = df['X'][i] - df['X'][0]
        y = df['Y'][i] - df['Y'][0]
        z = df['Z'][i] - df['Z'][0]
        qx = df['qx'][i]
        qy = df['qy'][i]
        qz = df['qz'][i]
        qw = df['qw'][i]
        T = np.identity(4)
        R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        T[0:3, 0:3] = R
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        
        poses.append((T))

    covariances = [covariance * np.eye(6) for _ in range(len(timestamps))]

    traj = norlab_trajectory.Trajectory(timestamps, poses, covariances)

    timestamps_interpolation = np.linspace(timestamps[0], timestamps[-1], len(timestamps)*points)

    for time in timestamps_interpolation:
        traj_estim.append(traj.getPose(time))
        
    values_x = [pose[0, 3] for pose in poses]
    values_y = [pose[1, 3] for pose in poses]
    values_z= [pose[2, 3] for pose in poses]
    
    if quiver:
        nx_u = [pose[0,0] for pose in poses]
        nx_v = [pose[1,0] for pose in poses]
        ny_u = [pose[0,1] for pose in poses]
        ny_v = [pose[1,1] for pose in poses]

    values_x_estim = [pose[0, 3] for pose in traj_estim]
    values_y_estim = [pose[1, 3] for pose in traj_estim]
    values_z_estim = [pose[2, 3] for pose in traj_estim]

    fig, ax = plt.subplots()
    ax.plot(values_x, values_y, '.', label='measured values')

    if quiver:
        ax.quiver(values_x, values_y, nx_u, nx_v, angles = 'xy', scale_units ='xy', scale =1 , headwidth =1, color='r', label='measured values')
        ax.quiver(values_x, values_y, ny_u, ny_v, angles = 'xy', scale_units ='xy', scale =1 , headwidth =1, color='g', label='measured values')

    plt.scatter(values_x_estim, values_y_estim, label='interpolated values', alpha=0.5, c='g')
    ax.set_aspect('equal')
    ax.legend()
    plt.show()

    if output:
        df = pd.DataFrame(columns=['Timestamp', 'X', 'Y', 'Z', 'qx', 'qy', 'qz', 'qw'])
        for i in range(len(timestamps_interpolation)):
            T = traj_estim[i]
            x = T[0, 3]
            y = T[1, 3]
            z = T[2, 3]
            R = Rotation.from_matrix(T[0:3, 0:3]).as_quat()
            qx = R[0]
            qy = R[1]
            qz = R[2]
            qw = R[3]
            df.loc[i] = [timestamps_interpolation[i], x, y, z, qx, qy, qz, qw]
        
        output = output if output.endswith('.csv') else f'{output}.csv'
        df.to_csv(OUTPUT_PATH / f'{output}', index=False)
    
def init_argparse():

    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input',
                        type=str, required=True,
                        help='Name of input file for the trajectory to interpolate.')
    parser.add_argument('-cov', '--covariance',
                        type=float, required=False, default=1e-7,
                        help='Covariance value for interpolation, default value is 1e-7.')
    parser.add_argument('-pt', '--points',
                        type=int, required=False, default=1,
                        help='Number of points between two timestamps, default value is 1.')
    parser.add_argument('-quiver', '--quiver',
                        type=bool, required=False, default=False,
                        help='Quiver plot for the normal vectors.')
    parser.add_argument('-o', '--output',
                        type=str, required=False, default=False,
                        help='Name of output file for the interpolated trajectory file.')
    return parser

if __name__ == '__main__' :
    parser = init_argparse()
    args = parser.parse_args()
    main(args.input, args.covariance, args.points, args.quiver, args.output)
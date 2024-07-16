import argparse
import os
import sys
from pathlib import Path

import pandas as pd
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from tqdm import tqdm


def main(bag_name, output_path, topic_name):
    """
    Extracts a trajectory from a rosbag file and saves it to a CSV file in TUM format.
    
    Parameters
    ----------
    bag_name : str
        Name of the input rosbag file.
    output_path : str
        Name of the output CSV file.
    topic_name : str
        Name of the topic to extract.
    
    Returns
    -------
    None
        The function saves the trajectory to a CSV file in TUM format.
    """

    input_filepath = bag_name

    if not os.path.isdir(input_filepath):
        print('Error: Cannot locate input bag file [%s]' % input_filepath, file=sys.stderr)
        sys.exit(2)
        
    with Reader(input_filepath) as reader:
        if topic_name not in reader.topics:
            print(f'Error: Cannot find topic {topic_name} in bag file {input_filepath}', file=sys.stderr)
        traj = {'timestamp': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}
        connections = [x for x in reader.connections if x.topic == topic_name]
        for conn, timestamp, data in tqdm(reader.messages(connections=connections)):
            try:
                msg = deserialize_cdr(data, conn.msgtype)
            except:
                print('Error: Unable to deserialize messages from desired topic.')
                sys.exit(3)
            traj['timestamp'].append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
            traj['x'].append(msg.pose.pose.position.x)
            traj['y'].append(msg.pose.pose.position.y)
            traj['z'].append(msg.pose.pose.position.z)
            traj['qx'].append(msg.pose.pose.orientation.x)
            traj['qy'].append(msg.pose.pose.orientation.y)
            traj['qz'].append(msg.pose.pose.orientation.z)
            traj['qw'].append(msg.pose.pose.orientation.w)

        df = pd.DataFrame(traj)
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        df.to_csv(output_path, index=False)
        print(f'Done. Saved trajectory to {output_path}')


def init_argparse():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input',
                        type=str, required=True,
                        help='Name of input rosbag.')
    parser.add_argument('-t', '--topic',
                        type=str, required=True,
                        help='Topic to save (need to be Odometry msgs).')
    parser.add_argument('-o', '--output',
                        type=str, required=True,
                        help='Output name for csv file in TUM format. Will be saved in ‘trajectories’ folder.')
    return parser
if __name__ == '__main__':
    parser = init_argparse()
    args = parser.parse_args()
    main(args.input, args.output, args.topic)
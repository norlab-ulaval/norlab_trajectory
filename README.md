# norlab_trajectory
This repository allow us to interpolate trajectories from localization algorithms using Gaussian Processes and STEAM (Simultaneous Trajectory Estimation and Mapping) optimization library made by ASRL (Autonomous Space Robotics Lab) of University of Toronto.

# Usage

Examples of how to use the library can be found in the test folder.
```
cd examples
python3 interpolation.py -i f1tenth.csv
```

To create a trajectory, you can use the `Trajectory(poses)` function. 
To interpolate a pose or a pose's covariance, you can use the query time timestamp and specify it to the `getPose(queryTime)` or `getPoseCovariance(queryTime)` functions.

To interpolated your own trajectory you need to use a tum format, you can use the `rosbag_to_tum_traj.py` in the script folder to convert a rosbag file topic to a tum trajectory file.
```
cd scripts
python3 rosbag_to_tum_traj.py -i <bag_name>.bag -t <topic_name> -o <trajectory_name>.csv
```

# Prerequisites

## Install using script

To install the library and its dependencies, you can use the `install.sh` script.
```
git clone --recurse-submodules git@github.com:norlab-ulaval/norlab_trajectory.git
./install.sh
```

## Install from source

Or you can install the library and its dependencies manually.

### Install C++ compiler, CMake

```````
sudo apt -q -y install build-essential cmake libomp-dev
```````

### Install eigen library

```````
sudo apt-get install libeigen3-dev
```````

### Install lgmath using ROS2

```````
cd ~/ros2_ws/src/
git clone https://github.com/norlab-ulaval/lgmath.git
```````
Or follow the instructions here : [https://github.com/norlab-ulaval/lgmath](https://github.com/norlab-ulaval/lgmath).

### Install steam using ROS2

```````
cd ~/ros2_ws/src/
git clone https://github.com/norlab-ulaval/steam.git
```````

Or follow the instructions here : [https://github.com/norlab-ulaval/steam](https://github.com/norlab-ulaval/steam).

### Install pybind11

pybind11 will allow us to use norlab_trajectory in python.
You can download the sources and CMake files as a Python package from pybind11 using pip3.
```````
pip3 install pybind11
```````
Or follow the instructions here : [https://github.com/pybind/pybind11](https://github.com/pybind/pybind11).

# Compilation
```````
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
```````
## Issues

If lgmath or steam are not found during the installation you can specify their location using :
```````
cd build
cmake-gui
```````
Specify the location using CMake flags to your installation directories. Then, generate the make files by clicking generate.

## References 
[1] Barfoot, T. D. and Furgale, P. T., “Associating Uncertainty with Three-Dimensional Poses for use in Estimation Problems,” IEEE Transactions on Robotics, 2014.

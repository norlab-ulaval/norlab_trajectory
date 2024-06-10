# norlab_trajectory
This repositery allow us to interpolate trajectories from localization algorithms using Gaussian Processes and STEAM (Simultaneous Trajectory Estimation and Mapping) optimization library made by ASRL (Autonomous Space Robotics Lab) of University of Toronto.

## Download

The source code is available from github, you can clone the git tree by doing:
```````
git clone git@github.com:norlab-ulaval/norlab_trajectory.git
```````

# Prerequisites

## Install C++ compiler, CMake

```````
sudo apt -q -y install build-essential cmake libomp-dev
```````

## Install eigen librairy

```````
sudo apt-get install libeigen3-dev
```````

## Install lgmath using ROS2

```````
cd ~/ros2_ws/src/
git clone https://github.com/norlab-ulaval/lgmath.git
```````
Or follow the instructions here : [https://github.com/norlab-ulaval/lgmath](https://github.com/norlab-ulaval/lgmath).

## Install steam using ROS2

```````
cd ~/ros2_ws/Src/
git clone https://github.com/norlab-ulaval/steam.git
```````

Or follow the instructions here : [https://github.com/norlab-ulaval/steam](https://github.com/norlab-ulaval/steam).

## Install pybind11

pybind11 will allow us to use norlab_trajectory in python.
You can download the sources and CMake files as a Python package from pybind11 using pip3.
```````
pip3 install pybind11
```````
Or follow the instructions here : [https://github.com/pybind/pybind11](https://github.com/pybind/pybind11).

## Compilation
```````
colcon build --symlink-install
source ../install/setup.bash
```````

## Usage

To use norlab_trajectory library you need to feed a list of tuples containing a timestamp and a 4x4 transformation matrix for each poses you want to interpolate using :
```````
.Trajectory(poses)
```````

And to interpolate a pose or a pose's covariance you can use the querytime timestamp and specify it to :
```````
.getPose(queryTime)
.getPoseCovariance(queryTime)
```````

## Issues

If lgmath or steam are not found during the installation you can specify their location using :
```````
cd build
cmake-gui
```````
Specify the location using CMake flags to your installation directories. Then, generate the make files by clicking generate.

# References 
[1] Barfoot, T. D. and Furgale, P. T., “Associating Uncertainty with Three-Dimensional Poses for use in Estimation Problems,” IEEE Transactions on Robotics, 2014.

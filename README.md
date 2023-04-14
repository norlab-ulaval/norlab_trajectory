# norlab_trajectory
This repositery allow us to interpolate trajectories from localisation algorithms using Gaussian Processes.

# Installation 

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
WORKSPACE=~/workspace  # choose your own workspace directory

mkdir -p ${WORKSPACE}/lgmath && cd $_
git clone https://github.com/utiasASRL/lgmath.git .

source <your ROS2 worspace>
colcon build --symlink-install --cmake-args "-DUSE_AMENT=ON"
```````
Or follow the instructions here : [https://github.com/utiasASRL/lgmath](https://github.com/utiasASRL/lgmath).

## Install steam using ROS2

```````
WORKSPACE=~/workspace  # choose your own workspace directory

mkdir -p ${WORKSPACE}/steam && cd $_
git clone https://github.com/utiasASRL/steam.git .

source <your ROS2 worspace that includes steam>
colcon build --symlink-install --cmake-args "-DUSE_AMENT=ON"
```````

Or follow the instructions here : [https://github.com/utiasASRL/steam](https://github.com/utiasASRL/steam).

# References 


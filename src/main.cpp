#include <iostream>
#include "Trajectory.h"

int main(int argc, char **argv) {
    Eigen::Matrix4f pose;
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    std::vector<std::pair<float, Eigen::Matrix4f>> poses;
    for (int i = 0; i < 10; i ++){
        pose(0, 3) = float(i);
        poses.emplace_back(i,pose);
    }
    Trajectory traj(poses);
        std::cout << "Hello World!" << std::endl;
    return 0;
}

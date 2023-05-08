#include <iostream>
#include "Trajectory.h"

int main(int argc, char** argv)
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    std::vector<std::pair<double, Eigen::Matrix4f>> poses;
    for(int i = 0; i < 10; i++)
    {
        pose(0, 3) = double(i);
        poses.emplace_back(i, pose);
    }
    Trajectory traj(poses);
    std::cout << "Hello World!" << std::endl;
    return 0;
}

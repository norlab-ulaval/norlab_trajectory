#ifndef NORLAB_TRAJECTORY_TRAJECTORY_H
#define NORLAB_TRAJECTORY_TRAJECTORY_H

#include <Eigen/Core>

class Trajectory
{
public:
    Trajectory(std::vector<std::pair<float, Eigen::Matrix4f>> poses);
    Eigen::Matrix4f getPose(float queryTime);
    Eigen::Matrix<float, 6, 6> getPoseCovariance(float queryTime);
};

#endif

#include "Trajectory.h"
#include <steam.hpp>

Trajectory::Trajectory(std::vector<std::pair<float, Eigen::Matrix4f>> poses)
{
}

Eigen::Matrix4f Trajectory::getPose(float queryTime)
{
}

Eigen::Matrix<float, 6, 6> Trajectory::getPoseCovariance(float queryTime)
{
}

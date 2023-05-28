#ifndef NORLAB_TRAJECTORY_TRAJECTORY_H
#define NORLAB_TRAJECTORY_TRAJECTORY_H

#include <steam.hpp>
#include <eigen3/Eigen/Core>

namespace norlab_trajectory
{
    class Trajectory
    {
    public:
        Trajectory(const std::vector<double>& timeStamps, const std::vector<Eigen::Matrix4f>& poses, const std::vector<Eigen::Matrix<float, 6, 6>>& covariances);
        Eigen::Matrix4f getPose(double queryTime);
        Eigen::Matrix<float, 12, 12> getPoseCovariance(double queryTime);
    private:
        steam::traj::const_vel::Interface traj;
        steam::OptimizationProblem problem;
        double firstPointTimeStamp;
    };
}

#endif

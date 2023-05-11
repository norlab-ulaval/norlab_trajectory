#ifndef NORLAB_TRAJECTORY_TRAJECTORY_H
#define NORLAB_TRAJECTORY_TRAJECTORY_H

#include <steam.hpp>
#include <eigen3/Eigen/Core>

class Trajectory
{
public:
    Trajectory(std::vector<std::pair<double, Eigen::Matrix4f>> poses);
    Eigen::Matrix4f getPose(double queryTime);
    Eigen::Matrix<float, 12, 12> getPoseCovariance(double queryTime);
private:
    steam::traj::const_vel::Interface traj;
    steam::OptimizationProblem problem;
    double firstPointTimeStamp;
};


#endif

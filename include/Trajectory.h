#ifndef NORLAB_TRAJECTORY_TRAJECTORY_H
#define NORLAB_TRAJECTORY_TRAJECTORY_H

#include <Eigen/Core>

class Trajectory
{
public:
    Trajectory(std::vector<std::pair<float, Eigen::Matrix4f>> poses);
    Eigen::Matrix4f getPose(float queryTime);
    Eigen::Matrix<float, 6, 1> getGeneralizedVelocity(float queryTime);
    Eigen::Matrix<float, 6, 6> getPoseCovariance(float queryTime);

private:
    Eigen::Matrix<float, 4, 4> hatOperator (Eigen::Matrix<float,6,1>);
    Eigen::Matrix<float, 6, 1> downHatOperator (Eigen::Matrix<float,4,4>);
    Eigen::Matrix<float, 6, 6> curlyHatOperator (Eigen::Matrix<float,6,1>);

};

#endif

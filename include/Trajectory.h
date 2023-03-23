#ifndef NORLAB_TRAJECTORY_TRAJECTORY_H
#define NORLAB_TRAJECTORY_TRAJECTORY_H

#include <Eigen/Core>
#include <Eigen/Dense>

class Trajectory
{
public:
    Trajectory(std::vector<std::pair<float, Eigen::Matrix4f>> poses);
    Eigen::Matrix4f getPose(float queryTime);
    Eigen::Matrix<float, 6, 1> getGeneralizedVelocity(float queryTime);
    Eigen::Matrix<float, 6, 6> getPoseCovariance(float queryTime);

private:
    static Eigen::Matrix<float, 3, 3> hatOperator_SO3(const Eigen::Matrix<float, 3, 1>& vector);
    static Eigen::Matrix<float, 4, 4> hatOperator(const Eigen::Matrix<float, 6, 1>& vector);
    static Eigen::Matrix<float, 3, 1> downHatOperator_SO3(const Eigen::Matrix<float, 3, 3>& matrix);
    static Eigen::Matrix<float, 6, 1> downHatOperator(const Eigen::Matrix<float, 4, 4>& matrix);
    static Eigen::Matrix<float, 6, 6> curlyHatOperator(const Eigen::Matrix<float, 6, 1>& vector);
    static Eigen::Matrix<float, 6, 1> downCurlyHatOperator(const Eigen::Matrix<float, 6, 6>& matrix);
    static Eigen::Matrix<float, 3, 3> computeQ(const Eigen::Matrix<float, 6, 1>& vector);
    static Eigen::Matrix<float, 6, 6> computeJ(const Eigen::Matrix<float, 6, 1>& vector);
    static Eigen::Matrix<float, 12, 12> computePhi(const Eigen::Matrix<float, 6, 1>& generalizedVelocity, const float& s, const float& t);
    static Eigen::Matrix<float, 12, 12> PowerSpectralDensity(const Eigen::Matrix<float, 6,6>& Qk, const Eigen::Matrix<float, 6, 1>& generalizedVelocity, const float& t, const float& s);
    static Eigen::Matrix<float, 12, 12> Lambda(const float& tau, const float& tk, const float& tkplus1, const Eigen::Matrix<float, 6,6>& Qk,const Eigen::Matrix<float, 6, 1>& generalizedVelocity, const Eigen::Matrix<float, 6, 6>& J);
    static Eigen::Matrix<float, 12, 12> Psi(const float& tau, const float& tk, const float& tkplus1, const Eigen::Matrix<float, 6,6>& Qk,const Eigen::Matrix<float, 6, 1>& generalizedVelocity, const Eigen::Matrix<float, 6, 6>& J);
    static Eigen::Matrix<float, 4, 4> exp4f(const Eigen::Matrix<float, 4, 4>& matrix);
    static Eigen::Matrix<float, 4, 4> ln(const Eigen::Matrix<float, 4, 4>& matrix);
    static Eigen::Matrix<float, 6, 6> exp6f(const Eigen::Matrix<float, 6, 6>& matrix);
    static Eigen::Matrix<float, 6, 6> ln(const Eigen::Matrix<float, 6, 6>& matrix);
};

#endif

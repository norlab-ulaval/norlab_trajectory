#include "Trajectory.h"

Trajectory::Trajectory(std::vector<std::pair<float, Eigen::Matrix4f>> poses)
{
}

Eigen::Matrix4f Trajectory::getPose(float queryTime)
{
}

Eigen::Matrix<float, 6, 1> Trajectory::getGeneralizedVelocity(float queryTime)
{
}

Eigen::Matrix<float, 6, 6> Trajectory::getPoseCovariance(float queryTime)
{
}

Eigen::Matrix3f Trajectory::hatOperator_SO3(const Eigen::Matrix<float, 3, 1>& vector)
{
    Eigen::Matrix3f result = Eigen::Matrix3f::Zero();
    result(0, 1) = -vector(2);
    result(0, 2) = vector(1);
    result(1, 0) = vector(2);
    result(1, 2) = -vector(0);
    result(2, 0) = -vector(1);
    result(2, 1) = vector(0);
    return result;
}

Eigen::Matrix4f Trajectory::hatOperator(const Eigen::Matrix<float, 6, 1>& vector)
{
    Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
    result(0, 3) = vector(0, 0);
    result(1, 3) = vector(1, 0);
    result(2, 3) = vector(2, 0);
    result.topLeftCorner<3, 3>() = hatOperator_SO3(vector.bottomRows<3>());
    return result;
}

Eigen::Matrix<float, 3, 1> Trajectory::downHatOperator_SO3(const Eigen::Matrix<float, 3, 3>& matrix)
{
    return {matrix(2, 1), matrix(0, 2), matrix(1, 0)};
}

Eigen::Matrix<float, 6, 1> Trajectory::downHatOperator(const Eigen::Matrix<float, 4, 4>& matrix)
{
    Eigen::Matrix<float, 6, 1> result;
    result.topRows<3>() = matrix.topRightCorner<3, 1>();
    result.bottomRows<3>() = downHatOperator_SO3(matrix.topLeftCorner<3, 3>());
    return result;
}

Eigen::Matrix<float, 6, 1> Trajectory::downCurlyHatOperator(const Eigen::Matrix<float, 6, 6>& matrix)
{
    Eigen::Matrix<float, 6, 1> result;
    result.topRows<3>() = downHatOperator_SO3(matrix.topRightCorner<3, 3>());
    result.bottomRows<3>() = downHatOperator_SO3(matrix.topLeftCorner<3, 3>());
    return result;
}

Eigen::Matrix<float, 6, 6> Trajectory::curlyHatOperator(const Eigen::Matrix<float, 6, 1>& vector)
{
    Eigen::Matrix<float, 3, 3> uHat = hatOperator_SO3(vector.topRows<3>());
    Eigen::Matrix<float, 3, 3> vHat = hatOperator_SO3(vector.bottomRows<3>());
    Eigen::Matrix<float, 6, 6> result = Eigen::Matrix<float, 6, 6>::Zero();
    result.topLeftCorner<3, 3>() = vHat;
    result.topRightCorner<3, 3>() = uHat;
    result.bottomRightCorner<3, 3>() = vHat;
    return result;
}

Eigen::Matrix<float, 3, 3> Trajectory::computeQ(const Eigen::Matrix<float, 6, 1>& vector)
{
    Eigen::Matrix<float, 3, 3> result;
    Eigen::Matrix<float, 3, 1> rho = vector.topRows<3>(); Eigen::Matrix<float, 3, 3> rhoHat = hatOperator_SO3(vector.topRows<3>());
    Eigen::Matrix<float, 3, 1> phi = vector.bottomRows<3>(); Eigen::Matrix<float, 3, 3> phiHat = hatOperator_SO3(vector.bottomRows<3>());
    float _phi = vector.bottomRows<3>().norm();
    result = ((1/2) * hatOperator_SO3(phi)) + ((_phi - std::sin(_phi)) / std::pow(_phi,3)) * (phiHat * rhoHat + rhoHat * phiHat + phiHat * rhoHat * phiHat)
            + ((std::pow(_phi, 2) + 2 * std::cos(_phi) - 2) / 2 * std::pow(_phi, 4)) * (phiHat * phiHat * rhoHat + rhoHat * phiHat * phiHat + 3 * phiHat * rhoHat * phiHat)
            + ((2 * _phi - 3 * std::sin(_phi) + _phi * std::cos(_phi)) / 2 * std::pow(_phi,5)) * (phiHat * rhoHat * phiHat * phiHat + phiHat * phiHat * rhoHat * phiHat);
    return result;
}

Eigen::Matrix<float,6,6> Trajectory::computeJ(const Eigen::Matrix<float,3,1>& vector,const Eigen::Matrix<float, 3, 3>& Q)
{
    Eigen::Matrix<float,3,3> J_SO3 = Eigen::Matrix3f::Identity() + (1/2) * hatOperator_SO3(vector);
    Eigen::Matrix< float, 6,6> result;
    result.topLeftCorner<3,3>() = J_SO3;
    result.topRightCorner<3,3>() = Q;
    result.bottomRightCorner<3,3>() = J_SO3;
    return result;
}

Eigen::Matrix<float, 12, 12> Trajectory::computePhi(const Eigen::Matrix<float, 6, 1>& generalizedVelocity, const Eigen::Matrix<float, 6, 6>& J, const float& s, const float& t)
{
    Eigen::Matrix<float, 12, 12> result = Eigen::Matrix<float,12,12>::Zero();

    result.topLeftCorner<6,6>() = exp6f((t-s) * curlyHatOperator(generalizedVelocity));
//    result.topRightCorner<6,6>() = Eigen::Matrix<float, 6, 6>::Ones() * (t-s) * J * (t-s) * generalizedVelocity;
    result.bottomRightCorner<6,6>() = Eigen::Matrix<float, 6, 6>::Ones();
    return result;
}

Eigen::Matrix<float, 4, 4> Trajectory::exp4f(const Eigen::Matrix<float, 4, 4>& matrix)
{
    float phi = downHatOperator(matrix).bottomRows<3>().norm();
    Eigen::Matrix3f V = Eigen::Matrix3f::Identity() + (1 - std::cos(phi)) / std::pow(phi, 2) * matrix.topLeftCorner<3, 3>() -
                        (phi - std::sin(phi)) / std::pow(phi, 3) * (matrix.topLeftCorner<3, 3>() * matrix.topLeftCorner<3, 3>());
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.topLeftCorner<3, 3>() =
            Eigen::Matrix3f::Identity() + std::sin(phi) / phi * matrix.topLeftCorner<3, 3>() +
            ((1 - std::cos(phi)) / std::pow(phi, 2)) * matrix.topLeftCorner<3, 3>() * matrix.topLeftCorner<3, 3>();
    result.topRightCorner<3, 1>() = V * matrix.topRightCorner<3, 1>();
    return result;
}

Eigen::Matrix<float, 4, 4> Trajectory::ln(const Eigen::Matrix<float, 4, 4>& matrix)
{
    float phi = std::acos((matrix.topLeftCorner<3, 3>().trace() - 1.0) / 2.0);
    Eigen::Matrix3f rotationLogarithm = phi / (2 * std::sin(phi)) * (matrix.topLeftCorner<3, 3>() - matrix.topLeftCorner<3, 3>().transpose());
    Eigen::Matrix3f V = Eigen::Matrix3f::Identity() + (1 - std::cos(phi)) / std::pow(phi, 2) * rotationLogarithm -
                        (phi - std::sin(phi)) / std::pow(phi, 3) * (rotationLogarithm * rotationLogarithm);
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.topLeftCorner<3, 3>() = rotationLogarithm;
    result.topRightCorner<3, 1>() = V.inverse() * matrix.topRightCorner<3, 1>();
    return result;
}

Eigen::Matrix<float, 6, 6> Trajectory::exp6f(const Eigen::Matrix<float, 6, 6>& matrix)
{
    Eigen::Matrix<float, 6, 1> zeta = downCurlyHatOperator(matrix);
    float phi = zeta.bottomRows<3>().norm();
    Eigen::Matrix3f V = Eigen::Matrix3f::Identity() + (1 - std::cos(phi)) / std::pow(phi, 2) * matrix.topLeftCorner<3, 3>() -
                        (phi - std::sin(phi)) / std::pow(phi, 3) * (matrix.topLeftCorner<3, 3>() * matrix.topLeftCorner<3, 3>());
    Eigen::Matrix<float, 6, 6> result = Eigen::Matrix<float, 6, 6>::Identity();
    Eigen::Matrix3f C = Eigen::Matrix3f::Identity() + std::sin(phi) / phi * matrix.topLeftCorner<3, 3>() +
                        ((1 - std::cos(phi)) / std::pow(phi, 2)) * matrix.topLeftCorner<3, 3>() * matrix.topLeftCorner<3, 3>();
    result.topLeftCorner<3, 3>() = C;
    result.topRightCorner<3, 3>() = hatOperator_SO3(V * zeta.topRows<3>()) * C;
    result.bottomRightCorner<3, 3>() = C;
    return result;
}

Eigen::Matrix<float, 6, 6> Trajectory::ln(const Eigen::Matrix<float, 6, 6>& matrix)
{
    float phi = std::acos((matrix.topLeftCorner<3, 3>().trace() - 1.0) / 2.0);
    Eigen::Matrix<float, 3, 1> r = downHatOperator_SO3(matrix.topRightCorner<3, 3>() * matrix.topLeftCorner<3, 3>().transpose());
    Eigen::Matrix3f rotationLogarithm = phi / (2 * std::sin(phi)) * (matrix.topLeftCorner<3, 3>() - matrix.topLeftCorner<3, 3>().transpose());
    Eigen::Matrix3f V = Eigen::Matrix3f::Identity() + (1 - std::cos(phi)) / std::pow(phi, 2) * rotationLogarithm -
                        (phi - std::sin(phi)) / std::pow(phi, 3) * (rotationLogarithm * rotationLogarithm);
    Eigen::Matrix<float, 6, 6> result = Eigen::Matrix<float, 6, 6>::Identity();
    result.topLeftCorner<3, 3>() = rotationLogarithm;
    result.topRightCorner<3, 3>() = hatOperator_SO3(V.inverse() * r);
    result.bottomRightCorner<3, 3>() = rotationLogarithm;
    return result;
}

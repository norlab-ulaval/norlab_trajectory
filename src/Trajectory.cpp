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

Eigen::Matrix<float, 4, 4> Trajectory::hatOperator(Eigen::Matrix<float, 6, 1>)
{

}

Eigen::Matrix<float, 6, 1> Trajectory::downHatOperator(Eigen::Matrix<float, 4, 4>)
{

}

Eigen::Matrix<float, 6, 6> Trajectory::curlyHatOperator(Eigen::Matrix<float, 6, 1>)
{

}

//topLeftCorner(nb lignes, nb colonnes) : retourne la sous matrice en haut a gauche (R)
//topRightCorner() : vecteur Translation
//initaliser une matrice

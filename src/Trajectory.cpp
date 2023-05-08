#include "Trajectory.h"

Trajectory::Trajectory(std::vector<std::pair<double, Eigen::Matrix4f>> poses):
        traj(Eigen::Matrix<double, 6, 1>::Ones())
{
    for(int i = 0; i < poses.size(); ++i)
    {
        Eigen::Matrix4d T = poses[i].second.cast<double>();
        std::shared_ptr<steam::se3::SE3StateVar> T_vi = steam::se3::SE3StateVar::MakeShared(lgmath::se3::Transformation(T));
        std::shared_ptr<steam::vspace::VSpaceStateVar<6>> w_iv_inv = steam::vspace::VSpaceStateVar<6>::MakeShared(Eigen::Matrix<double, 6, 1>::Zero());

        traj.add(steam::traj::Time(poses[i].first), T_vi, w_iv_inv);
        problem.addStateVariable(T_vi);
        problem.addStateVariable(w_iv_inv);

        std::shared_ptr<steam::se3::SE3ErrorEvaluator> errorFunc = steam::se3::SE3ErrorEvaluator::MakeShared(T_vi, lgmath::se3::Transformation(T));
        std::shared_ptr<steam::StaticNoiseModel<6>> noiseModel = steam::StaticNoiseModel<6>::MakeShared(Eigen::Matrix<double, 6, 6>::Identity() * 100000,
                                                                                                        steam::NoiseType::INFORMATION);
        std::shared_ptr<steam::L2LossFunc> lossFunc = steam::L2LossFunc::MakeShared();
        std::shared_ptr<steam::WeightedLeastSqCostTerm<6>> costTerm = steam::WeightedLeastSqCostTerm<6>::MakeShared(errorFunc, noiseModel, lossFunc);
        problem.addCostTerm(costTerm);
    }
    traj.addPriorCostTerms(problem);

    steam::GaussNewtonSolver::Params params;
    params.verbose = true;
    steam::GaussNewtonSolver solver(problem, params);
    solver.optimize();
}

Eigen::Matrix4f Trajectory::getPose(float queryTime)
{
    return traj.getPoseInterpolator(queryTime)->value().matrix().cast<float>();
}

Eigen::Matrix<float, 6, 6> Trajectory::getPoseCovariance(float queryTime)
{
    steam::Covariance covariance(problem);
    Eigen::Matrix4d pose = getPose(queryTime).cast<double>();
    std::shared_ptr<steam::se3::SE3StateVar> pose_state_var = std::make_shared<steam::se3::SE3StateVar>(lgmath::se3::Transformation(pose));
    return covariance.query(pose_state_var).cast<float>();
}
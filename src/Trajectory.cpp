#include "Trajectory.h"

Trajectory::Trajectory(std::vector<std::pair<float, Eigen::Matrix4f>> poses) {
    for (int i = 0; i < poses.size(); ++i) {
        Eigen::Matrix4d T = poses[i].second.cast<double>();
        std::shared_ptr<steam::se3::SE3StateVar> T_vi = std::make_shared<steam::se3::SE3StateVar>(
                lgmath::se3::Transformation(T));
        T_vi->locked() = true;
        std::shared_ptr<steam::vspace::VSpaceStateVar<6>> w_iv_inv = std::make_shared<steam::vspace::VSpaceStateVar<6>>(
                Eigen::Matrix<double, 6, 1>::Zero());
        traj.add(poses[i].first, T_vi, w_iv_inv);
        problem.addStateVariable(T_vi);
        problem.addStateVariable(w_iv_inv);
        Eigen::Vector3d reference = Eigen::Vector3d::Zero();
//        std::shared_ptr<steam::p2p::P2PErrorEvaluator> error_func = std::make_shared<steam::p2p::P2PErrorEvaluator>(
//                T_vi, reference.transpose(), poses[i].second.topRightCorner<3, 1>().cast<double>());
//        std::shared_ptr<steam::StaticNoiseModel<3>> noiseModel = std::make_shared<steam::StaticNoiseModel<3>>(
//                Eigen::Matrix3d::Identity(), steam::NoiseType::INFORMATION);
//        std::shared_ptr<steam::L2LossFunc> lossFunc = std::make_shared<steam::L2LossFunc>();
//        std::shared_ptr<steam::WeightedLeastSqCostTerm<3>> costTerm = std::make_shared<steam::WeightedLeastSqCostTerm<3>>(
//                error_func, noiseModel, lossFunc);
//        problem.addCostTerm(costTerm);
    }
//    steam::GaussNewtonSolver::Params params;
//    steam::GaussNewtonSolver solver(problem,params);
//    solver.optimize();
}

Eigen::Matrix4f Trajectory::getPose(float queryTime) {
    return traj.getPoseInterpolator(queryTime)->value().matrix().cast<float>();
}

Eigen::Matrix<float, 6, 6> Trajectory::getPoseCovariance(float queryTime) {
    steam::Covariance covariance(problem);
    Eigen::Matrix4d pose = getPose(queryTime).cast<double>();
    std::shared_ptr<steam::se3::SE3StateVar> pose_state_var = std::make_shared<steam::se3::SE3StateVar>(lgmath::se3::Transformation(pose));
    return covariance.query(pose_state_var).cast<float>();
}
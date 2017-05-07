//
//  loop_closure_factor.cpp
//  VINS_ios
//
//  Created by 栗大人 on 2017/3/2.
//  Copyright © 2017年 栗大人. All rights reserved.
//

#include "loop_closure_factor.hpp"

Eigen::Matrix2d LoopClosureFactor::sqrt_info;
double LoopClosureFactor::sum_t;

LoopClosureFactor::LoopClosureFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                                     const Eigen::Vector3d &_old_p, const Eigen::Quaterniond &_old_q, const double _loss_ratio)
: pts_i(_pts_i), pts_j(_pts_j),
old_p(_old_p), old_q(_old_q),loss_ratio(_loss_ratio)
{
};

bool LoopClosureFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    
    Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    
    double inv_dep_i = parameters[2][0];
    
    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = old_q.inverse() * (pts_w - old_p);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    
    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();

    
    residual = loss_ratio * sqrt_info * residual;
    
    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d R_old = old_q.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
        0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        reduce = loss_ratio * sqrt_info * reduce;
        
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            
            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * R_old.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * R_old.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);
            
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }
        
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric.transpose() * (R_old.transpose() * Ri - Eigen::Matrix3d::Identity());
            Eigen::Matrix3d tmp_r = ric.transpose() * R_old.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) + Utility::skewSymmetric(tmp_r * pts_camera_i) +
            Utility::skewSymmetric(ric.transpose() * (R_old.transpose() * (Ri * tic + Pi - old_p) - tic));
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[2]);
            jacobian_feature = reduce * ric.transpose() * R_old.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);

        }
    }
    return true;
}

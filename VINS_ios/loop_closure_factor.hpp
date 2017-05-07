//
//  loop_closure_factor.hpp
//  VINS_ios
//
//  Created by 栗大人 on 2017/3/2.
//  Copyright © 2017年 栗大人. All rights reserved.
//

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility.hpp"
#include "global_param.hpp"

class LoopClosureFactor : public ceres::SizedCostFunction<2, 7, 7, 1>
{
public:
    LoopClosureFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                      const Eigen::Vector3d &_old_p, const Eigen::Quaterniond &_old_q, const double _loss_ratio);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d old_p;
    Eigen::Quaterniond old_q;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
    double loss_ratio;
};

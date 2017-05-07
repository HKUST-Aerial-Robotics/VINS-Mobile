//
//  projection_facor.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef projection_facor_hpp
#define projection_facor_hpp

#include <stdio.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility.hpp"

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
public:
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
        
    Eigen::Vector3d pts_i, pts_j;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

#endif /* projection_facor_hpp */

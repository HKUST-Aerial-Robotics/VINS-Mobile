//
//  perspective_facor.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef perspective_facor_hpp
#define perspective_facor_hpp

#include <stdio.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility.hpp"

class PerspectiveFactor : public ceres::SizedCostFunction<2, 7, 7>
{
public:
    PerspectiveFactor(const Eigen::Vector2d &_pts_2d, const Eigen::Vector3d &_pts_3d, const int _track_num);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
    Eigen::Vector3d pts_3d;
    Eigen::Vector2d pts_2d;
    int track_num;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

#endif /* perspective_facor_hpp */

//
//  pose_local_parameterization.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef pose_local_parameterization_hpp
#define pose_local_parameterization_hpp

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "utility.hpp"

class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};

#endif /* pose_local_parameterization_hpp */

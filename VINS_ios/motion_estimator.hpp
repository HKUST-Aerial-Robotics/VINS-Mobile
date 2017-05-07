//
//  motion_estimator.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/12/26.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef motion_estimator_hpp
#define motion_estimator_hpp

#include <stdio.h>
#include <vector>
using namespace std;
#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include "utility.hpp"
using namespace Eigen;


class MotionEstimator
{
public:
    
    bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);
    
private:
    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};
#endif /* motion_estimator_hpp */

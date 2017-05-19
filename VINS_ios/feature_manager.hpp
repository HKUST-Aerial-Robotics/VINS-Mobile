//
//  feature_manager.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/20.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef feature_manager_hpp
#define feature_manager_hpp

#include <stdio.h>
#include <list>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include "feature_tracker.hpp"
#include <eigen3/Eigen/Dense>
#include "global_param.hpp"
#include "utility.hpp"

#define COMPENSATE_ROTATION false
#define MIN_PARALLAX_POINT ((double)(3.0/549))
#define MIN_PARALLAX ((double)(10.0/549))
#define INIT_DEPTH ((double)(5.0))

using namespace Eigen;
using namespace std;
using namespace std;
class FeaturePerFrame
{
public:
    FeaturePerFrame(const Vector3d &_point)
    {
        z = _point(2);
        point = _point / z;
    }
    Vector3d point;
    double z;
    bool is_used;
    double parallax;
    double dep_gradient;
};

class FeaturePerId
{
public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;
    
    int used_num;
    bool is_margin;
    bool is_outlier;
    
    double estimated_depth;
    
    bool fixed;
    bool in_point_cloud;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;
    
    FeaturePerId(int _feature_id, int _start_frame)
    : feature_id(_feature_id), start_frame(_start_frame),
    used_num(0), estimated_depth(-1.0),is_outlier(false),fixed(false),in_point_cloud(false)
    {
    }
    
    int endFrame();
};

class FeatureManager
{
public:
    FeatureManager(Matrix3d _Rs[]);
    bool addFeatureCheckParallax(int frame_count, const map<int, Vector3d> &image_msg, int &parallax_num);
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    void triangulate(Vector3d Ps[], Vector3d tic, Matrix3d ric, bool is_nonlinear);
    VectorXd getDepthVector();
    
    int getFeatureCount();
    void clearState();
    void tagMarginalizedPoints(bool marginalization_flag);
    void removeBack();
    void removeFront(int frame_count);
    void setDepth(const VectorXd &x);
    void clearDepth(const VectorXd &x);
    void shift(int n_start_frame);
    void removeFailures();
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    /*
     variables
     */
    list<FeaturePerId> feature;
    std::vector<std::pair<int, std::vector<int>>> outlier_info;
    int last_track_num;
    
private:
    double compensatedParallax1(FeaturePerId &it_per_id);
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric;
    
};
#endif /* feature_manager_hpp */

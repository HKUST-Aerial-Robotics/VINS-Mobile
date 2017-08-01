//
//  feature_tracker.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef feature_tracker_hpp
#define feature_tracker_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "global_param.hpp"
#include <string>
#include <list>
#include "utility.hpp"
#include <opencv2/core/eigen.hpp>
#include "vins_pnp.hpp"

#define MAX_CNT 70
#define MIN_DIST 30
#define COL 480
#define ROW 640
#define F_THRESHOLD 1.0
#define EQUALIZE 1
using namespace cv;
using namespace std;
using namespace Eigen;
/*
 image frame
 --------> x:480
 |
 |
 |
 |
 |
 | y:640
 */
struct max_min_pts{
    Point2f min;
    Point2f max;
};

struct IMU_MSG_LOCAL {
    double header;
    Vector3d acc;
    Vector3d gyr;
};

class FeatureTracker
{
public:
    FeatureTracker();
    bool solveVinsPnP(double header, Vector3d &P, Matrix3d &R, bool vins_normal);
    void readImage(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<Point2f> &good_pts, vector<double> &track_len, double header, Vector3d &P, Matrix3d &R, bool vins_normal);
    void setMask();
    void rejectWithF();
    void addPoints();
    bool updateID(unsigned int i);
    
    /*
     varialbles
     */
    int frame_cnt;
    cv::Mat mask;
    cv::Mat cur_img, pre_img, forw_img;
    
    vector<cv::Point2f> n_pts,cur_pts,pre_pts,forw_pts;
    
    vector<int> ids,track_cnt;
    vector<max_min_pts> parallax_cnt;
    static int n_id;
    int img_cnt;
    double current_time;
    vinsPnP vins_pnp;
    bool use_pnp;
    
    /*
     interface
     */
    map<int, Vector3d> image_msg;
    bool update_finished;
    list<IMG_MSG_LOCAL> solved_features;
    VINS_RESULT solved_vins;
    vector<IMU_MSG_LOCAL> imu_msgs;
    
};
#endif /* feature_tracker_hpp */

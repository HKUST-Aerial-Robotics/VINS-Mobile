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

#define MAX_CNT 60
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

class FeatureTracker
{
  public:
    FeatureTracker();
    void FindFeatures(const cv::Mat &_img, cv::Mat &result);
    void readImage(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<Point2f> &good_pts, vector<double> &track_len);
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
    
    /*
     interface
    */
    map<int, Vector3d> image_msg;
    bool update_finished;

};
#endif /* feature_tracker_hpp */

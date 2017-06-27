//
//  VINS.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef VINS_hpp
#define VINS_hpp

#include <stdio.h>
#include "feature_manager.hpp"
#include "utility.hpp"
#include "projection_facor.hpp"
#include "pose_local_parameterization.hpp"
#include "global_param.hpp"
#include <ceres/ceres.h>
#include "marginalization_factor.hpp"
#include "imu_factor.h"
#include "draw_result.hpp"
#include <opencv2/core/eigen.hpp>
#include "inital_sfm.hpp"
#include "initial_aligment.hpp"
#include "motion_estimator.hpp"

extern bool LOOP_CLOSURE;
struct RetriveData
{
    /* data */
    int old_index;
    int cur_index;
    double header;
    Vector3d P_old;
    Quaterniond Q_old;
    Vector3d P_cur;
    Quaterniond Q_cur;
    vector<cv::Point2f> measurements;
    vector<int> features_ids;
    bool use;
    Vector3d relative_t;
    Quaterniond relative_q;
    double relative_yaw;
    double loop_pose[7];
};

class VINS
{
public:
    
    typedef IMUFactor IMUFactor_t;
    
    VINS();
    
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
    
    FeatureManager f_manager;
    MotionEstimator m_estimator;
    int frame_count;
    
    Matrix3d ric;
    Vector3d tic;
    MarginalizationFlag  marginalization_flag;
    Vector3d Ps[10 * (WINDOW_SIZE + 1)];
    Vector3d Vs[10 * (WINDOW_SIZE + 1)];
    Matrix3d Rs[10 * (WINDOW_SIZE + 1)];
    Vector3d Bas[10 * (WINDOW_SIZE + 1)];
    Vector3d Bgs[10 * (WINDOW_SIZE + 1)];
    
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    
    //for loop closure
    RetriveData retrive_pose_data, front_pose;
    bool loop_enable;
    vector<Vector3f> correct_point_cloud;
    Vector3f correct_Ps[WINDOW_SIZE];
    Matrix3f correct_Rs[WINDOW_SIZE];
    Vector3d t_drift;
    Matrix3d r_drift;
    
    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;
    vector<Vector3f> point_cloud;
    
    int feature_num;
    
    IntegrationBase *pre_integrations[10 * (WINDOW_SIZE + 1)];
    bool first_imu;
    Vector3d acc_0, gyr_0;
    vector<double> dt_buf[10 * (WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[10 * (WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[10 * (WINDOW_SIZE + 1)];
    Matrix<double, 7, 1> IMU_linear[10 * (WINDOW_SIZE + 1)];
    Matrix3d IMU_angular[10 * (WINDOW_SIZE + 1)];
    double Headers[10 * (WINDOW_SIZE + 1)];
    Vector3d g;
    
    vector<Vector3d> init_poses;
    double initial_timestamp;
    Vector3d init_P;
    Vector3d init_V;
    Matrix3d init_R;
    
    SolverFlag solver_flag;
    Matrix3d Rc[10 * (WINDOW_SIZE + 1)];
    
    //for initialization
    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;
    Matrix3d back_R0;
    Vector3d back_P0;
    //for falure detection
    bool failure_hand;
    bool failure_occur;
    Matrix3d last_R, last_R_old;
    Vector3d last_P, last_P_old;
    
    //for visulization
    DrawResult drawresult;
    cv::Mat image_show;
    cv::Mat imageAI;
    enum InitStatus
    {
        FAIL_IMU,
        FAIL_PARALLAX,
        FAIL_RELATIVE,
        FAIL_SFM,
        FAIL_PNP,
        FAIL_ALIGN,
        FAIL_CHECK,
        SUCC
    };
    InitStatus init_status;
    int parallax_num_view;
    int fail_times;
    int initProgress;
    double final_cost;
    double visual_cost;
    int visual_factor_num;
    
    void solve_ceres(int buf_num);
    void solveCalibration();
    void old2new();
    void new2old();
    void clearState();
    void setIMUModel();
    void setExtrinsic();
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void processImage(map<int, Vector3d> &image_msg, double header, int buf_num);
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void changeState();
    bool solveInitial();
    bool relativePose(int camera_id, Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool visualInitialAlign();
    bool failureDetection();
    void failureRecover();
    void reInit();
    void update_loop_correction();
};
#endif /* VINS_hpp */

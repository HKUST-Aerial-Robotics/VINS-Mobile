//
//  global_param.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/20.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef global_param_h
#define global_param_h

enum DeviceType
{
    iPhone7P,
    iPhone7,
    iPhone6sP,
    iPhone6s,
    iPadPro97,
    iPadPro129,
    iPadMini7,
    unDefine
};
//extrinsic param
// RIC_YPR=(-1.408, -3.930, -171.871)
// RIC_YPR=(-2.031, -6.447, -169.227)
// RIC_YPR=(1.203, -7.616, -169.068)
// RIC_YPR=(-0.758, -8.325, -169.167)
// RIC_YPR=(1.198, -5.853, -170.326)
#define RIC_y ((double)1.198)
#define RIC_p ((double)-5.853)
#define RIC_r ((double)-170.326)
#define MIN_LOOP_NUM 22
#define LOOP_FREQ 3
#define WINDOW_SIZE 10
#define PNP_SIZE 6
#define SIZE_POSE 7
#define SIZE_SPEEDBIAS 9
#define SIZE_SPEED 3
#define SIZE_BIAS  6

#define SIZE_FEATURE 1

#define NUM_OF_F 1000
#define NUM_OF_CAM 1
#define DEBUG_MODE false

// Temporary calibration mode for camera-IMU extrinsics.
// Set ENABLE_EXTRINSIC_CALIBRATION to true to optimize para_Ex_Pose online.
// Use the two toggles below to optimize translation and rotation separately.
#define ENABLE_EXTRINSIC_CALIBRATION false
#define CALIBRATE_EXTRINSIC_TRANSLATION true
#define CALIBRATE_EXTRINSIC_ROTATION true
#define EXTRINSIC_CALIB_LOG_EVERY_N_SOLVES 10

// Dampen extrinsic optimizer increments in local parameterization.
// 1.0 = default Ceres step, 0.1 = 10x smaller updates.
#define EXTRINSIC_TRANSLATION_STEP_SCALE ((double)0.05)
#define EXTRINSIC_ROTATION_STEP_SCALE ((double)0.05)

// Optional hard cap on per-iteration increment magnitudes.
// Set <= 0 to disable each cap.
#define EXTRINSIC_TRANSLATION_MAX_STEP_M ((double)0.001)
#define EXTRINSIC_ROTATION_MAX_STEP_RAD ((double)0.25 * C_PI / 180.0)

#define C_PI 3.1415926

#define GRAVITY ((double)9.805)
#define ACC_N ((double)0.5)  //0.02
#define ACC_W ((double)0.002)
#define GYR_N ((double)0.2)  //0.02
#define GYR_W ((double)4.0e-5)
#define BIAS_ACC_THRESHOLD ((double)0.5)
#define BIAS_GYR_THRESHOLD ((double)0.1)
#define G_THRESHOLD ((double)3.0)
#define G_NORM ((double)9.805)
#define INIT_KF_THRESHOLD ((double)18)
#define SFM_R_THRESHOLD ((double)180)
#define MAX_FEATURE_CNT 150

extern double FOCUS_LENGTH_Y;
extern double PY;
extern double FOCUS_LENGTH_X;
extern double PX;
extern double SOLVER_TIME;
extern int FREQ;

//extrinsic param
extern double TIC_X;
extern double TIC_Y;
extern double TIC_Z;
/* IMU
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 
 */
enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

bool setGlobalParam(DeviceType device);

#if 1
#define TS(name) int64 t_##name = cv::getTickCount()
#define TE(name) printf("TIMER_" #name ": %.2fms\n", \
1000.*((cv::getTickCount() - t_##name) / cv::getTickFrequency()))
#else
#define TS(name)
#define TE(name)
#endif

#endif /* global_param_h */


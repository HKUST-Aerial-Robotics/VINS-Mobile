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
    unDefine
};
//extrinsic param
#define RIC_y ((double)0.0)
#define RIC_p ((double)0.0)
#define RIC_r ((double)180.0)
#define MIN_LOOP_NUM 20
#define LOOP_FREQ 3
#define WINDOW_SIZE 10
#define SIZE_POSE 7
#define SIZE_SPEEDBIAS  9

#define SIZE_FEATURE 1

#define NUM_OF_F 1000
#define NUM_OF_CAM 1
#define DEBUG_MODE false
#define C_PI 3.1414926

#define GRAVITY ((double)9.805)
#define ACC_N ((double)0.5)  //0.02
#define ACC_W ((double)0.002)
#define GYR_N ((double)0.2)  //0.02
#define GYR_W ((double)4.0e-5)
#define BIAS_ACC_THRESHOLD ((double)0.5)
#define BIAS_GYR_THRESHOLD ((double)0.1)
#define G_THRESHOLD ((double)1.0)
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


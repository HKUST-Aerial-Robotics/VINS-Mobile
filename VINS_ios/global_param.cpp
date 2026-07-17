//
//  global_param.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2017/05/09.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include <stdio.h>
#include "global_param.hpp"

double FOCUS_LENGTH_Y;
double PY;
double FOCUS_LENGTH_X;
double PX;
double SOLVER_TIME;
int FREQ;

//extrinsic param
double TIC_X;
double TIC_Y;
double TIC_Z;

bool setGlobalParam(DeviceType device)
{
    switch (device) {
        case iPhone7P:
            printf("Device iPhone7 plus param\n");
            FOCUS_LENGTH_X = 526.600;
            FOCUS_LENGTH_Y = 526.678;
            PX = 243.481;
            PY = 315.280;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            return true;
            break;
            
        case iPhone7:
            printf("Device iPhone7 param\n");
            FOCUS_LENGTH_X = 526.958;
            FOCUS_LENGTH_Y = 527.179;
            PX = 244.473;
            PY = 313.844;
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            return true;
            break;
            
        case iPhone6s:
            printf("Device iPhone6s param\n");
            FOCUS_LENGTH_Y = 549.477;
            PY = 320.379;
            FOCUS_LENGTH_X = 548.813;
            PX = 238.520;
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.065;
            TIC_Z = 0.0;
            return true;
            break;
            
        case iPhone6sP:
            printf("Device iPhone6sP param\n");
            FOCUS_LENGTH_X = 547.565;
            FOCUS_LENGTH_Y = 547.998;
            PX = 239.033;
            PY = 309.452;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.065;
            TIC_Z = 0.0;
            return true;
            break;
            
        case iPadPro97:
            printf("Device ipad97 param\n");
            FOCUS_LENGTH_X = 547.234;
            FOCUS_LENGTH_Y = 547.464;
            PX = 241.549;
            PY = 317.957;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.1;
            return true;
            break;
            
        case iPadPro129:
            printf("Device iPad129 param\n");
            FOCUS_LENGTH_X = 547.234;
            FOCUS_LENGTH_Y = 547.464;
            PX = 241.549;
            PY = 317.957;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.1;
            return true;
            break;
        case iPadMini7:
            printf("Device ipadmini7 param\n");
            FOCUS_LENGTH_X = 556.4297;
            FOCUS_LENGTH_Y = 556.4297;
            PX = 239.5;
            PY = 319.5;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            //extrinsic param
            // previous estimates
            // TIC=(0.069707, 0.138924, -0.186578)
            // TIC=(0.540468, 0.626427, -1.507500)
            // TIC=(0.327627, 0.619361, -1.370088)
            // TIC=(0.063635, 0.232340, -0.463219)
            // TIC=(0.021857, 0.138471, 0.008501) 
            // TIC=(0.137622, 0.331454, -0.582089)
            // TIC=(0.180273, 0.348541, -0.467740)
            // TIC=(0.157694, 0.281995, -0.321626)
            TIC_X = 0.157694;
            TIC_Y = 0.281995;
            TIC_Z = -0.321626;
            return true;
            break;
        case unDefine:
            printf("Device unDefine param\n");
            return false;
            break;
        default:
            printf("Device %d param\n", device);
            return false;
            break;
    }
}






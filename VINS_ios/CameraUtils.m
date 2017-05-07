//
//  CameraUtils.m
//  VINS_ios
//
//  Created by Yang Liu on 3/18/17.
//  Copyright © 2017 栗大人. All rights reserved.
//

#import "CameraUtils.h"
#import <AVFoundation/AVFoundation.h>

@implementation CameraUtils

+ (void)setExposureOffset:(float)ev {
    AVCaptureDevice *device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    NSLog(@"Camera: Get device %@", device);
    
    
//    NSLog(@"Camera: (before) minExp %f, maxExp %f",
//          CMTimeGetSeconds(device.activeVideoMinFrameDuration),
//          CMTimeGetSeconds(device.activeVideoMaxFrameDuration));
    
    NSLog(@"Camera: activeFormat %@", [device activeFormat]);
    
    NSError * error;
    [device lockForConfiguration:&error];
    if (error != nil) {
        NSLog(@"Camera: Error %@", error);
        return;
    }
    
    NSLog(@"Camera: exposure duration (before EV) %f", CMTimeGetSeconds(device.exposureDuration));
    [device setExposureTargetBias:ev completionHandler:^(CMTime syncTime) {
        NSLog(@"timestamp %lf", syncTime);
        NSLog(@"Camera: exposure duration (after EV) %f", CMTimeGetSeconds(device.exposureDuration));
    }];
    
//    device.exposureMode = AVCaptureExposureModeCustom;
    
//    [device setExposureModeCustomWithDuration:CMTimeMakeWithSeconds(0.01, 1000*1000*1000)
//                                          ISO:AVCaptureISOCurrent
//                            completionHandler: ^(CMTime syncTime){
//                                        NSLog(@"Camera: exposure duration (after EV) %f", CMTimeGetSeconds(device.exposureDuration));
//                                    }];
    
//    NSLog(@"Camera: (after) minExp %f, maxExp %f",
//          CMTimeGetSeconds(device.activeVideoMinFrameDuration),
//          CMTimeGetSeconds(device.activeVideoMaxFrameDuration));
    
    [device unlockForConfiguration];
}

@end

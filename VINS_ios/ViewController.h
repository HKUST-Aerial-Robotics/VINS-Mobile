//
//  ViewController.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/videoio/cap_ios.h>
#import "feature_tracker.hpp"
#import <mach/mach_time.h> 
#import "global_param.hpp"
#import "VINS.hpp"
#include <queue>
#import "draw_result.hpp"
#import <CoreMotion/CoreMotion.h>
#include "keyframe.h"
#include "loop_closure.h"
#include "keyfame_database.h"
#import <sys/utsname.h>

@interface ViewController : UIViewController<CvVideoCameraDelegate,UITextViewDelegate>
{
    CvVideoCamera* videoCamera;
    BOOL isCapturing;
    cv::Ptr<FeatureTracker> feature_tracker;
    cv::Size frameSize;
    uint64_t prevTime;
    NSCondition *_condition;
    NSThread *mainLoop;
    NSThread *draw;
    NSThread *saveData;
    NSThread *loop_thread;
    NSThread *globalLoopThread;
    UITextView *textY;
}

@property (nonatomic, strong) CvVideoCamera* videoCamera;
@property (nonatomic, strong) IBOutlet UIImageView* imageView;
@property (nonatomic, strong) IBOutlet UIImageView* featureImageView;


@property (weak, nonatomic) IBOutlet UIButton *stopButton;
@property (weak, nonatomic) IBOutlet UIButton *startButton;

@property (weak, nonatomic) IBOutlet UIButton *recordButton;
@property (weak, nonatomic) IBOutlet UIButton *playbackButton;

- (IBAction)recordButtonPressed:(id)sender;
- (IBAction)playbackButtonPressed:(id)sender;

-(IBAction)startButtonPressed:(id)sender;
-(IBAction)stopButtonPressed:(id)sender;
@property (weak, nonatomic) IBOutlet UISegmentedControl *switchUI;

- (void)showInputView;

struct IMU_MSG {
    NSTimeInterval header;
    Vector3d acc;
    Vector3d gyr;
};

struct IMG_MSG {
    NSTimeInterval header;
    map<int, Vector3d> point_clouds;
};

struct IMG_DATA {
    NSTimeInterval header;
    UIImage *image;
};

struct IMG_DATA_CACHE {
    NSTimeInterval header;
    cv::Mat equ_image;
    UIImage *image;
};
struct VINS_DATA_CACHE {
    NSTimeInterval header;
    Vector3f P;
    Matrix3f R;
};

typedef shared_ptr <IMU_MSG const > ImuConstPtr;
typedef shared_ptr <IMG_MSG const > ImgConstPtr;
@end


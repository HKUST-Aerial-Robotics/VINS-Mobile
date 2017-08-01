//
//  ViewController.m
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#import "ViewController.h"
#import "utility.hpp"
#import "CameraUtils.h"

@interface ViewController ()
@property (weak, nonatomic) IBOutlet UILabel *X_label;
@property (weak, nonatomic) IBOutlet UILabel *Y_label;
@property (weak, nonatomic) IBOutlet UILabel *Z_label;
@property (weak, nonatomic) IBOutlet UILabel *buf_label;
@property (weak, nonatomic) IBOutlet UILabel *total_odom_label;
@property (weak, nonatomic) IBOutlet UILabel *loop_label;
@property (weak, nonatomic) IBOutlet UILabel *feature_label;
@property (weak, nonatomic) IBOutlet UILabel *feature_label2;
@property (weak, nonatomic) IBOutlet UILabel *feature_label3;
@property (weak, nonatomic) IBOutlet UISlider *fovSlider;
@property (weak, nonatomic) IBOutlet UILabel *fovLabel;
@end

@implementation ViewController

/*************************** Save data for debug ***************************/

bool start_record = false;

bool start_playback = false;

bool start_playback_vins = false;

unsigned long imageDataIndex = 0;

unsigned long imageDataReadIndex = 0;

unsigned long imuDataIndex = 0;

unsigned long imuDataReadIndex = 0;

unsigned long vinsDataIndex = 0;

unsigned long vinsDataReadIndex = 0;

queue<IMG_DATA> imgDataBuf;

NSMutableData *imuDataBuf = [[NSMutableData alloc] init];

NSData *imuReader;

NSMutableData *vinsDataBuf = [[NSMutableData alloc] init];

NSData *vinsReader;

IMG_DATA imgData;

IMU_MSG imuData;

KEYFRAME_DATA vinsData;

/*************************** Save data for debug ***************************/

/******************************* UI CONFIG *******************************/

// false:  VINS trajectory is the main view, AR image is in left bottom
// true: AR image is the main view, VINS is in left bottom
bool ui_main = false;

bool box_in_AR = false;

bool box_in_trajectory = false;

// If initialized finished, start show is true
bool start_show = false;

// Indicate the initialization progress rate
UIActivityIndicatorView *indicator;

// Used for show VINS trajectory and AR
@synthesize imageView;

// Used for show initialization UI
@synthesize featureImageView;

@synthesize videoCamera;

// Used for show alert if vocabulary is not ready
UIAlertView *alertView;

// Textview for showing vins status
int loop_old_index = -1;

float x_view_last = -5000;

float y_view_last = -5000;

float z_view_last = -5000;

float total_odom = 0;

/******************************* UI CONFIG *******************************/

FeatureTracker featuretracker;

VINS vins;

// Store the fesature data processed by featuretracker
queue<ImgConstPtr> img_msg_buf;

// Store the IMU data for vins
queue<ImuConstPtr> imu_msg_buf;

// Store the IMU data for motion-only vins
queue<IMU_MSG_LOCAL> local_imu_msg_buf;

// The number of measurements waiting to be processed
int waiting_lists = 0;

int frame_cnt = 0;

// Lock the feature and imu data buffer
std::mutex m_buf;

std::condition_variable con;

NSTimeInterval current_time = -1;

NSTimeInterval lateast_imu_time = -1;

int imu_prepare = 0;

// MotionManager for read imu data
CMMotionManager *motionManager;

// Segment the trajectory using color when re-initialize
int segmentation_index = 0;

// Set true:  30 HZ pose output and AR rendering in front-end (very low latency)
// Set false: 10 HZ pose output and AR rendering in back-end
bool USE_PNP = false;

// Lock the solved VINS data feedback to featuretracker
std::mutex m_depth_feedback;

// Lock the IMU data feedback to featuretracker
std::mutex m_imu_feedback;

// Solved VINS feature feedback to featuretracker
list<IMG_MSG_LOCAL> solved_features;

// Solved VINS status feedback to featuretracker
VINS_RESULT solved_vins;

/******************************* Loop Closure ******************************/

// Raw image data buffer for extracting FAST feature
queue<pair<cv::Mat, double>> image_buf_loop;

// Lock the image_buf_loop
std::mutex m_image_buf_loop;

// Detect loop
LoopClosure *loop_closure;

// Keyframe database
KeyFrameDatabase keyframe_database;

// Control the loop detection frequency
int keyframe_freq = 0;

// Index the keyframe
int global_frame_cnt = 0;

// Record the checked loop frame
int loop_check_cnt = 0;

// Indicate if breif vocabulary read finish
bool voc_init_ok = false;

// Indicate the loop frame index
int old_index = -1;

// Translation drift
Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);

// Rotation drift
Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();

/******************************* Loop Closure ******************************/

// MARK: Unity Camera Mode Switching
// Ground truth from UI switch property "self.switchUIAREnabled"

// Implied, updated by updateCameraMode()
bool imuPredictEnabled = false;

// Implied, updated by updateCameraMode()
bool cameraMode = true;

// Implied, updated by updateCameraMode()
bool imageCacheEnabled = cameraMode && !USE_PNP;


// MARK: ViewController Methods

- (void)viewDidLoad {
    [super viewDidLoad];
    
    /*******************************************Camera setup*******************************************/
    self.videoCamera = [[CvVideoCamera alloc]
                        initWithParentView:imageView];
    
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition =
    AVCaptureDevicePositionBack;
    
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    self.videoCamera.defaultAVCaptureSessionPreset =
    AVCaptureSessionPreset640x480;
#ifdef DATA_EXPORT
    self.videoCamera.defaultFPS = 1;
#else
    self.videoCamera.defaultFPS = 30;
#endif
    
    isCapturing = NO;
    
    [CameraUtils setExposureOffset: -1.0f];
    [videoCamera start];
    
    /***************************************UI configuration*****************************************/
    UIPanGestureRecognizer *resultPanGestureRecognizer = [[UIPanGestureRecognizer alloc]
                                                          initWithTarget:self
                                                          action:@selector(handlePan:)];
    resultPanGestureRecognizer.minimumNumberOfTouches = 1;
    resultPanGestureRecognizer.maximumNumberOfTouches = 2;
    [self.imageView addGestureRecognizer:resultPanGestureRecognizer];
    
    UIPinchGestureRecognizer *resultPinchGestureRecognizer = [[UIPinchGestureRecognizer alloc]
                                                              initWithTarget:self
                                                              action:@selector(handlePinch:)];
    [self.imageView addGestureRecognizer:resultPinchGestureRecognizer];
    
    UITapGestureRecognizer *resultTapGestureRecognizer = [[UITapGestureRecognizer alloc]
                                                          initWithTarget:self
                                                          action:@selector(handleTap:)];
    [self.imageView addGestureRecognizer:resultTapGestureRecognizer];
    
    UILongPressGestureRecognizer *resultLongPressGestureRecognizer = [[UILongPressGestureRecognizer alloc]
                                                                      initWithTarget:self
                                                                      action:@selector(handleLongPress:)];
    [self.imageView addGestureRecognizer:resultLongPressGestureRecognizer];
    
    if (!feature_tracker)
        feature_tracker = new FeatureTracker();
    
    //give projection variance
    vins.setIMUModel();
    
    //UI
    _loopButton.layer.zPosition = 1;
    _reinitButton.layer.zPosition = 1;
    alertView = [[UIAlertView alloc]initWithTitle:@"WARN" message:@"please wait for vocabulary loading!"
                                         delegate:self cancelButtonTitle:@"confirm" otherButtonTitles:@"cancel", nil];
    
    indicator = [[UIActivityIndicatorView alloc] initWithActivityIndicatorStyle:UIActivityIndicatorViewStyleWhiteLarge];
    indicator.center = CGPointMake(self.imageView.frame.size.width * 0.5, self.imageView.frame.size.height * 0.22);
    indicator.color = [UIColor darkGrayColor];
    [indicator startAnimating];
    [self.view addSubview:indicator];
    
    /****************************************Init all the thread****************************************/
    _condition=[[NSCondition alloc] init];
    mainLoop=[[NSThread alloc]initWithTarget:self selector:@selector(run) object:nil];
    [mainLoop setName:@"mainLoop"];
    
    saveData=[[NSThread alloc]initWithTarget:self selector:@selector(saveData) object:nil];
    [saveData setName:@"saveData"];
    
    if(LOOP_CLOSURE)
    {
        //loop closure thread
        loop_thread = [[NSThread alloc]initWithTarget:self selector:@selector(loop_thread) object:nil];
        [loop_thread setName:@"loop_thread"];
        [loop_thread start];
        
        globalLoopThread=[[NSThread alloc]initWithTarget:self selector:@selector(globalLoopThread) object:nil];
        [globalLoopThread setName:@"globalLoopThread"];
        [globalLoopThread start];
    }
    
    /************************************Device and iOS version check************************************/
    bool deviceCheck = setGlobalParam(deviceName());
    if(!deviceCheck)
    {
        UIAlertController *alertDevice = [UIAlertController alertControllerWithTitle:@"Unsupported Device"
                                                                             message:@"Supported devices are: iPhone7 Plus, iPhone7, iPhone6s Plus, iPhone6s" preferredStyle:UIAlertControllerStyleAlert];
        
        UIAlertAction *okAction = [UIAlertAction
                                   actionWithTitle:@"OK"
                                   style:UIAlertActionStyleDefault
                                   handler:^(UIAlertAction * _Nonnull action) {
                                       
                                   }];
        
        [alertDevice addAction:okAction];
        
        dispatch_async(dispatch_get_main_queue(), ^ {
            [self presentViewController:alertDevice animated:YES completion:nil];
        });
    }
    vins.setExtrinsic();
    vins.setIMUModel();
    featuretracker.vins_pnp.setExtrinsic();
    featuretracker.vins_pnp.setIMUModel();
    bool versionCheck = iosVersion();
    if(!versionCheck)
    {
        UIAlertController *alertVersion = [UIAlertController alertControllerWithTitle:@"Warn"
                                                                              message:@"Please upgrade your iOS version!" preferredStyle:UIAlertControllerStyleAlert];
        UIAlertAction *cancelAction = [UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:^(UIAlertAction * action)
                                       {exit(0);}];
        UIAlertAction *okAction = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action)
                                   {exit(0);}];
        [alertVersion addAction:cancelAction];
        [alertVersion addAction:okAction];
        dispatch_async(dispatch_get_main_queue(), ^ {
            [self presentViewController:alertVersion animated:YES completion:nil];
        });
    }
    
    /*********************************************Start VINS*******************************************/
    if(versionCheck && deviceCheck)
    {
        [self imuStartUpdate];
        isCapturing = YES;
        [mainLoop start];
        motionManager = [[CMMotionManager alloc] init];
        frameSize = cv::Size(videoCamera.imageWidth,
                             videoCamera.imageHeight);
    }
}

/*
 Main process image thread: this thread detects and track feature between two continuous images
 and takes the newest VINS result and the corresponding image to draw AR and trajectory.
 */
queue<IMG_DATA_CACHE> image_pool;
queue<VINS_DATA_CACHE> vins_pool;
IMG_DATA_CACHE image_data_cache;
cv::Mat lateast_equa;
UIImage *lateast_image;
Vector3f lateast_P;
Matrix3f lateast_R;

cv::Mat pnp_image;
Vector3d pnp_P;
Matrix3d pnp_R;

- (void)processImage:(cv::Mat&)image
{
    if(isCapturing == YES)
    {
        //NSLog(@"image processing");
        float lowPart = image.at<float>(0,0);  //modify opencv library, timestamp was stored at index 0,0
        float highPart = image.at<float>(0,1);
        //image.at<float>(0,0) = image.at<float>(1,0);
        //image.at<float>(0,1) = image.at<float>(1,1);
        shared_ptr<IMG_MSG> img_msg(new IMG_MSG());
        //cout << (videoCamera->grayscaleMode) << endl;
        //img_msg->header = [[NSDate date] timeIntervalSince1970];
        img_msg->header = [[NSProcessInfo processInfo] systemUptime];
        float Group[2];
        Group[0] = lowPart;
        Group[1] = highPart;
        double* time_now_decode = (double*)Group;
        double time_stamp = *time_now_decode;
        
        if(lateast_imu_time <= 0)
        {
            cv::cvtColor(image, image, CV_BGRA2RGB);
            cv::flip(image,image,-1);
            return;
        }
        //img_msg->header = lateast_imu_time;
        img_msg->header = time_stamp;
        BOOL isNeedRotation = image.size() != frameSize;
        
        //for save data
        cv::Mat input_frame;
        if(start_playback)
        {
            //TS(readImg);
            bool still_play;
            still_play = [self readImageTime:imageDataReadIndex];
            [self readImage:imageDataReadIndex];
            if(!still_play)
                return;
            imageDataReadIndex++;
#ifdef DATA_EXPORT
            [self tapSaveImageToIphone:imgData.image];
#endif
            UIImageToMat(imgData.image,image);
            UIImageToMat(imgData.image,input_frame);
            img_msg->header = imgData.header;
            //TE(readImg);
#ifdef DATA_EXPORT
            printf("record play image: %lf\n",imgData.header,imageDataReadIndex);
#endif
        }
        else
        {
            input_frame = image;
        }
        
        if(start_record)
        {
            imgData.header = img_msg->header;
            imgData.image = MatToUIImage(image);
            imgDataBuf.push(imgData);
            return;
        }
        else
        {
            if(!imgDataBuf.empty())
                return;
        }
        
        prevTime = mach_absolute_time();
        
        cv::Mat gray;
        cv::cvtColor(input_frame, gray, CV_RGBA2GRAY);
        cv::Mat img_with_feature;
        cv::Mat img_equa;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3);
        clahe->apply(gray, img_equa);
        //img_equa = gray;
        TS(time_feature);
        
        m_depth_feedback.lock();
        featuretracker.solved_features = solved_features;
        featuretracker.solved_vins = solved_vins;
        m_depth_feedback.unlock();
        
        m_imu_feedback.lock();
        featuretracker.imu_msgs = getImuMeasurements(img_msg->header);
        m_imu_feedback.unlock();
        
        vector<Point2f> good_pts;
        vector<double> track_len;
        bool vins_normal = (vins.solver_flag == VINS::NON_LINEAR);
        featuretracker.use_pnp = USE_PNP;
        featuretracker.readImage(img_equa, img_with_feature,frame_cnt, good_pts, track_len, img_msg->header, pnp_P, pnp_R, vins_normal);
        TE(time_feature);
        //cvtColor(img_equa, img_equa, CV_GRAY2BGR);
        for (int i = 0; i < good_pts.size(); i++)
        {
            cv::circle(image, good_pts[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
        }
        
        //image msg buf
        if(featuretracker.img_cnt==0)
        {
            img_msg->point_clouds = featuretracker.image_msg;
            //img_msg callback
            m_buf.lock();
            img_msg_buf.push(img_msg);
            //NSLog(@"Img timestamp %lf",img_msg_buf.front()->header);
            m_buf.unlock();
            con.notify_one();
            if(imageCacheEnabled)
            {
                image_data_cache.header = img_msg->header;
                image_data_cache.image = MatToUIImage(image);
                image_pool.push(image_data_cache);
            }
            
            if(LOOP_CLOSURE)
            {
                m_image_buf_loop.lock();
                cv::Mat loop_image = gray.clone();
                image_buf_loop.push(make_pair(loop_image, img_msg->header));
                if(image_buf_loop.size() > WINDOW_SIZE)
                    image_buf_loop.pop();
                m_image_buf_loop.unlock();
            }
        }
        
        featuretracker.img_cnt = (featuretracker.img_cnt + 1) % FREQ;
        for (int i = 0; i < good_pts.size(); i++)
        {
            cv::circle(image, good_pts[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
        }
        TS(visualize);
        if(imageCacheEnabled)
        {
            //use aligned vins and image
            if(!vins_pool.empty() && !image_pool.empty())
            {
                while(vins_pool.size() > 1)
                {
                    vins_pool.pop();
                }
                while(!image_pool.empty() && image_pool.front().header < vins_pool.front().header)
                {
                    image_pool.pop();
                }
                if(!vins_pool.empty() && !image_pool.empty())
                {
                    lateast_image = image_pool.front().image;
                    lateast_P = vins_pool.front().P;
                    lateast_R = vins_pool.front().R;
                    UIImageToMat(lateast_image, image);
                }
            }
            else if(!image_pool.empty())
            {
                if(image_pool.size() > 10)
                    image_pool.pop();
            }
        }
        if(USE_PNP)
        {
            lateast_P = pnp_P.cast<float>();
            lateast_R = pnp_R.cast<float>();
        }
        if(ui_main || start_show == false || vins.solver_flag != VINS::NON_LINEAR)  //show image and AR
        {
            cv::Mat tmp2;
            if(vins.solver_flag == VINS::NON_LINEAR && start_show)
            {
                cv::Mat tmp;
                vins.drawresult.startInit = true;
                vins.drawresult.drawAR(vins.imageAI, vins.correct_point_cloud, lateast_P, lateast_R);
                
                cv::cvtColor(image, tmp, CV_RGBA2BGR);
                cv::Mat mask;
                cv::Mat imageAI = vins.imageAI;
                if(!imageAI.empty())
                    cv::cvtColor(imageAI, mask, CV_RGB2GRAY);
                imageAI.copyTo(tmp,mask);
                cv::cvtColor(tmp, image, CV_BGRA2BGR);
            }
            if(DEBUG_MODE)
            {
                cv::flip(lateast_equa, image, -1);
            }
            else
            {
                cv::flip(image,tmp2,-1);
                image = tmp2;
                if(vins.solver_flag != VINS::NON_LINEAR || !start_show)
                    cv::cvtColor(image, image, CV_RGBA2BGR);
            }
        }
        else //show VINS
        {
            if(vins.solver_flag == VINS::NON_LINEAR)
            {
                vins.drawresult.pose.clear();
                vins.drawresult.pose = keyframe_database.refine_path;
                vins.drawresult.segment_indexs = keyframe_database.segment_indexs;
                vins.drawresult.Reprojection(vins.image_show, vins.correct_point_cloud, vins.correct_Rs, vins.correct_Ps, box_in_trajectory);
            }
            cv::Mat tmp2 = vins.image_show;
            
            cv::Mat down_origin_image;
            cv::resize(image.t(), down_origin_image, cv::Size(200, 150));
            cv::cvtColor(down_origin_image, down_origin_image, CV_BGRA2RGB);
            cv::flip(down_origin_image,down_origin_image,0);
            cv::Mat imageROI;
            imageROI = tmp2(cv::Rect(10,COL - down_origin_image.rows- 10, down_origin_image.cols,down_origin_image.rows));
            cv::Mat mask;
            cv::cvtColor(down_origin_image, mask, CV_RGB2GRAY);
            down_origin_image.copyTo(imageROI, mask);
            
            
            cv::cvtColor(tmp2, image, CV_BGRA2BGR);
            cv::flip(image,tmp2,1);
            if (isNeedRotation)
                image = tmp2.t();
        }
        
        TE(visualize);
    } else {
        // Not capturing, means not started yet
        cv::cvtColor(image, image, CV_BGRA2RGB);
        cv::flip(image,image,-1);
        //BOOL isNeedRotation = image.size() != frameSize;
        //if (isNeedRotation)
        //    image = image.t();
    }
}


/*
 Send imu data and visual data into VINS
 */
std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    while (true)
    {
        if (imu_msg_buf.empty() || img_msg_buf.empty())
            return measurements;
        
        if (!(imu_msg_buf.back()->header > img_msg_buf.front()->header))
        {
            NSLog(@"wait for imu, only should happen at the beginning");
            return measurements;
        }
        
        if (!(imu_msg_buf.front()->header < img_msg_buf.front()->header))
        {
            NSLog(@"throw img, only should happen at the beginning");
            img_msg_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = img_msg_buf.front();
        img_msg_buf.pop();
        
        std::vector<ImuConstPtr> IMUs;
        while (imu_msg_buf.front()->header <= img_msg->header)
        {
            IMUs.emplace_back(imu_msg_buf.front());
            imu_msg_buf.pop();
        }
        //NSLog(@"IMU_buf = %d",IMUs.size());
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

vector<IMU_MSG_LOCAL> getImuMeasurements(double header)
{
    vector<IMU_MSG_LOCAL> imu_measurements;
    static double last_header = -1;
    if(last_header < 0 || local_imu_msg_buf.empty())
    {
        last_header = header;
        return imu_measurements;
    }
    
    while(!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= last_header)
        local_imu_msg_buf.pop();
    while(!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= header)
    {
        imu_measurements.emplace_back(local_imu_msg_buf.front());
        local_imu_msg_buf.pop();
    }
    last_header = header;
    return imu_measurements;
}

void send_imu(const ImuConstPtr &imu_msg)
{
    NSTimeInterval t = imu_msg->header;
    if (current_time < 0)
        current_time = t;
    double dt = (t - current_time);
    current_time = t;
    
    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};
    
    double dx = imu_msg->acc.x() - ba[0];
    double dy = imu_msg->acc.y() - ba[1];
    double dz = imu_msg->acc.z() - ba[2];
    
    double rx = imu_msg->gyr.x() - bg[0];
    double ry = imu_msg->gyr.y() - bg[1];
    double rz = imu_msg->gyr.z() - bg[2];
    //NSLog(@"IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);
    
    vins.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

/*
 VINS thread: this thread tightly fuses the visual measurements and imu data and solves pose, velocity, IMU bias, 3D feature for all frame in WINNDOW
 If the newest frame is keyframe, then push it into keyframe database
 */
-(void)run{
    [_condition lock];
    while (![[NSThread currentThread] isCancelled])
    {
        [self process];
        [NSThread sleepForTimeInterval:0.01];
    }
    [_condition unlock];
    
}

int kf_global_index;
bool start_global_optimization = false;
-(void)process
{
    
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    std::unique_lock<std::mutex> lk(m_buf);
    con.wait(lk, [&]
             {
                 return (measurements = getMeasurements()).size() != 0;
             });
    lk.unlock();
    waiting_lists = measurements.size();
    for(auto &measurement : measurements)
    {
        for(auto &imu_msg : measurement.first)
        {
            send_imu(imu_msg);
        }
        
        auto img_msg = measurement.second;
        map<int, Vector3d> image = img_msg->point_clouds;
        //NSLog(@"Image timestamp = %lf",img_msg->header);
        double header = img_msg->header;
        TS(process_image);
        vins.processImage(image,header,waiting_lists);
        TE(process_image);
        double time_now = [[NSProcessInfo processInfo] systemUptime];
        double time_vins = vins.Headers[WINDOW_SIZE];
        NSLog(@"vins delay %lf", time_now - time_vins);
        
        //update feature position for front-end
        if(vins.solver_flag == vins.NON_LINEAR)
        {
            m_depth_feedback.lock();
            solved_vins.header = vins.Headers[WINDOW_SIZE - 1];
            solved_vins.Ba = vins.Bas[WINDOW_SIZE - 1];
            solved_vins.Bg = vins.Bgs[WINDOW_SIZE - 1];
            solved_vins.P = vins.correct_Ps[WINDOW_SIZE-1].cast<double>();
            solved_vins.R = vins.correct_Rs[WINDOW_SIZE-1].cast<double>();
            solved_vins.V = vins.Vs[WINDOW_SIZE - 1];
            Vector3d R_ypr = Utility::R2ypr(solved_vins.R);
            solved_features.clear();
            for (auto &it_per_id : vins.f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;
                if (it_per_id.solve_flag != 1)
                    continue;
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                IMG_MSG_LOCAL tmp_feature;
                tmp_feature.id = it_per_id.feature_id;
                tmp_feature.position = vins.r_drift * vins.Rs[imu_i] * (vins.ric * pts_i + vins.tic) + vins.r_drift * vins.Ps[imu_i] + vins.t_drift;
                tmp_feature.track_num = (int)it_per_id.feature_per_frame.size();
                solved_features.push_back(tmp_feature);
            }
            m_depth_feedback.unlock();
        }
        
        if(imageCacheEnabled)
        {
            //add state into vins buff for alignwith image
            if(vins.solver_flag == VINS::NON_LINEAR && start_show)
            {
                VINS_DATA_CACHE vins_data_cache;
                vins_data_cache.header = vins.Headers[WINDOW_SIZE-1];
                vins_data_cache.P = vins.correct_Ps[WINDOW_SIZE-1];
                vins_data_cache.R = vins.correct_Rs[WINDOW_SIZE-1];
                vins_pool.push(vins_data_cache);
            }
            else if(vins.failure_occur == true)
            {
                vins.drawresult.change_color = true;
                vins.drawresult.indexs.push_back(vins.drawresult.pose.size());
                segmentation_index++;
                keyframe_database.max_seg_index++;
                keyframe_database.cur_seg_index = keyframe_database.max_seg_index;
                
                while(!vins_pool.empty())
                    vins_pool.pop();
            }
        }
        /**
         *** start build keyframe database for loop closure
         **/
        if(LOOP_CLOSURE)
        {
            static bool first_frame = true;
            if(vins.solver_flag != vins.NON_LINEAR)
                first_frame = true;
            if(vins.marginalization_flag == vins.MARGIN_OLD && vins.solver_flag == vins.NON_LINEAR && !image_buf_loop.empty())
            {
                first_frame = false;
                if(!first_frame && keyframe_freq % LOOP_FREQ == 0)
                {
                    keyframe_freq = 0;
                    /**
                     ** save the newest keyframe to the keyframe database
                     ** only need to save the pose to the keyframe database
                     **/
                    Vector3d T_w_i = vins.Ps[WINDOW_SIZE - 2];
                    Matrix3d R_w_i = vins.Rs[WINDOW_SIZE - 2];
                    m_image_buf_loop.lock();
                    while(!image_buf_loop.empty() && image_buf_loop.front().second < vins.Headers[WINDOW_SIZE - 2])
                    {
                        image_buf_loop.pop();
                    }
                    //assert(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second);
                    if(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second)
                    {
                        const char *pattern_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_pattern" ofType:@"yml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
                        KeyFrame* keyframe = new KeyFrame(vins.Headers[WINDOW_SIZE - 2], global_frame_cnt, T_w_i, R_w_i, image_buf_loop.front().first, pattern_file, keyframe_database.cur_seg_index);
                        keyframe->setExtrinsic(vins.tic, vins.ric);
                        /*
                         ** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
                         ** and save the pointcloud to the keyframe for reprojection search correspondance
                         */
                        keyframe->buildKeyFrameFeatures(vins);
                        keyframe_database.add(keyframe);
                        
                        global_frame_cnt++;
                    }
                    m_image_buf_loop.unlock();
                    
                }
                else
                {
                    first_frame = false;
                }
                // update loop info
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(vins.Headers[i] == vins.front_pose.header)
                    {
                        KeyFrame* cur_kf = keyframe_database.getKeyframe(vins.front_pose.cur_index);
                        if (abs(vins.front_pose.relative_yaw) > 30.0 || vins.front_pose.relative_t.norm() > 10.0)
                        {
                            printf("Wrong loop\n");
                            cur_kf->removeLoop();
                            break;
                        }
                        cur_kf->updateLoopConnection(vins.front_pose.relative_t,
                                                     vins.front_pose.relative_q,
                                                     vins.front_pose.relative_yaw);
                        break;
                    }
                }
                /*
                 ** update the keyframe pose when this frame slides out the window and optimize loop graph
                 */
                int search_cnt = 0;
                for(int i = 0; i < keyframe_database.size(); i++)
                {
                    search_cnt++;
                    KeyFrame* kf = keyframe_database.getLastKeyframe(i);
                    if(kf->header == vins.Headers[0])
                    {
                        kf->updateOriginPose(vins.Ps[0], vins.Rs[0]);
                        //update edge
                        // if loop happens in this frame, update pose graph;
                        if (kf->has_loop)
                        {
                            kf_global_index = kf->global_index;
                            start_global_optimization = true;
                        }
                        break;
                    }
                    else
                    {
                        if(search_cnt > 2 * WINDOW_SIZE)
                            break;
                    }
                }
                keyframe_freq++;
            }
        }
        waiting_lists--;
        
        //finish solve one frame
        [self performSelectorOnMainThread:@selector(showInputView) withObject:nil waitUntilDone:YES];
    }
}


/*
 Loop detection thread: this thread detect loop for newest keyframe and retrieve features
 */
-(void)loop_thread{
    
    if(LOOP_CLOSURE && loop_closure == NULL)
    {
        NSLog(@"loop start load voc");
        TS(load_voc);
        const char *voc_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_k10L6" ofType:@"bin"]
                                cStringUsingEncoding:[NSString defaultCStringEncoding]];
        loop_closure = new LoopClosure(voc_file, COL, ROW);
        TE(load_voc);
        NSLog(@"loop load voc finish");
        
        voc_init_ok = true;
    }
    while(![[NSThread currentThread] isCancelled] )
    {
        if(!LOOP_CLOSURE)
        {
            [NSThread sleepForTimeInterval:0.5];
            continue;
        }
        
        bool loop_succ = false;
        if (loop_check_cnt < global_frame_cnt)
        {
            KeyFrame* cur_kf = keyframe_database.getLastKeyframe();
            //assert(loop_check_cnt == cur_kf->global_index);
            loop_check_cnt++;
            cur_kf->check_loop = 1;
            
            cv::Mat current_image;
            current_image = cur_kf->image;
            
            std::vector<cv::Point2f> measurements_old;
            std::vector<cv::Point2f> measurements_old_norm;
            std::vector<cv::Point2f> measurements_cur;
            std::vector<int> features_id;
            std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;
            
            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            cur_kf->extractBrief(current_image);
            printf("loop extract %d feature\n", cur_kf->keypoints.size());
            loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
            if(loop_succ)
            {
                KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
                if (old_kf == NULL)
                {
                    printf("NO such %dth frame in keyframe_database\n", old_index);
                    assert(false);
                }
                printf("loop succ with %drd image\n", old_index);
                assert(old_index!=-1);
                
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                
                old_kf->getPose(T_w_i_old, R_w_i_old);
                cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                   measurements_old, measurements_old_norm);
                measurements_cur = cur_kf->measurements;
                features_id = cur_kf->features_id;
                
                if(measurements_old_norm.size()>MIN_LOOP_NUM)
                {
                    
                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm;
                    retrive_data.features_ids = features_id;
                    vins.retrive_pose_data = (retrive_data);
                    
                    //cout << "old pose " << T_w_i_old.transpose() << endl;
                    //cout << "refinded pose " << T_w_i_refine.transpose() << endl;
                    // add loop edge in current frame
                    cur_kf->detectLoop(old_index);
                    keyframe_database.addLoop(old_index);
                    old_kf->is_looped = 1;
                    loop_old_index = old_index;
                }
            }
            cur_kf->image.release();
        }
        
        if(loop_succ)
            [NSThread sleepForTimeInterval:2.0];
        [NSThread sleepForTimeInterval:0.05];
    }
    //[self process_loop_detection];
}

/*
 GLobal Pose graph thread: optimize global pose graph based on realative pose from vins and update the keyframe database
 */
-(void)globalLoopThread{
    while (![[NSThread currentThread] isCancelled])
    {
        if(start_global_optimization)
        {
            start_global_optimization = false;
            TS(loop_thread);
            keyframe_database.optimize4DoFLoopPoseGraph(kf_global_index,
                                                        loop_correct_t,
                                                        loop_correct_r);
            vins.t_drift = loop_correct_t;
            vins.r_drift = loop_correct_r;
            TE(loop_thread);
            [NSThread sleepForTimeInterval:1.17];
        }
        [NSThread sleepForTimeInterval:0.03];
    }
}

/*
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 IMU data process and interploration
 
 */
bool imuDataFinished = false;
bool vinsDataFinished = false;
shared_ptr<IMU_MSG> cur_acc(new IMU_MSG());
vector<IMU_MSG> gyro_buf;  // for Interpolation
- (void)imuStartUpdate
{
    CMMotionManager *motionManager = [[CMMotionManager alloc] init];
    if (!motionManager.accelerometerAvailable) {
        NSLog(@"没有加速计");
    }
#ifdef DATA_EXPORT
    motionManager.accelerometerUpdateInterval = 0.1;
    motionManager.gyroUpdateInterval = 0.1;
#else
    motionManager.accelerometerUpdateInterval = 0.01;
    motionManager.gyroUpdateInterval = 0.01;
#endif
    
    [motionManager startDeviceMotionUpdates];
    
    [motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue]
                                        withHandler:^(CMAccelerometerData *latestAcc, NSError *error)
     {
         double header = motionManager.deviceMotion.timestamp;
         motionManager.deviceMotion.attitude.roll * 180.0 / M_PI,  //pitch for vins
         motionManager.deviceMotion.attitude.pitch * 180.0 / M_PI;  //roll for vins
         if(imu_prepare<10)
         {
             imu_prepare++;
         }
         shared_ptr<IMU_MSG> acc_msg(new IMU_MSG());
         acc_msg->header = latestAcc.timestamp;
         acc_msg->acc << -latestAcc.acceleration.x * GRAVITY,
         -latestAcc.acceleration.y * GRAVITY,
         -latestAcc.acceleration.z * GRAVITY;
         cur_acc = acc_msg;
         //printf("imu acc update %lf %lf %lf %lf\n", acc_msg->header, acc_msg->acc.x(), acc_msg->acc.y(), acc_msg->acc.z());
         
     }];
    [motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMGyroData *latestGyro, NSError *error)
     {
         //The time stamp is the amount of time in seconds since the device booted.
         NSTimeInterval header = latestGyro.timestamp;
         if(header<=0)
             return;
         if(imu_prepare < 10)
             return;
         
         IMU_MSG gyro_msg;
         gyro_msg.header = header;
         gyro_msg.gyr << latestGyro.rotationRate.x,
         latestGyro.rotationRate.y,
         latestGyro.rotationRate.z;
         
         if(gyro_buf.size() == 0)
         {
             gyro_buf.push_back(gyro_msg);
             gyro_buf.push_back(gyro_msg);
             return;
         }
         else
         {
             gyro_buf[0] = gyro_buf[1];
             gyro_buf[1] = gyro_msg;
         }
         //interpolation
         shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
         if(cur_acc->header >= gyro_buf[0].header && cur_acc->header < gyro_buf[1].header)
         {
             imu_msg->header = cur_acc->header;
             imu_msg->acc = cur_acc->acc;
             imu_msg->gyr = gyro_buf[0].gyr + (cur_acc->header - gyro_buf[0].header)*(gyro_buf[1].gyr - gyro_buf[0].gyr)/(gyro_buf[1].header - gyro_buf[0].header);
             //printf("imu gyro update %lf %lf %lf\n", gyro_buf[0].header, imu_msg->header, gyro_buf[1].header);
             //printf("imu inte update %lf %lf %lf %lf\n", imu_msg->header, gyro_buf[0].gyr.x(), imu_msg->gyr.x(), gyro_buf[1].gyr.x());
         }
         else
         {
             printf("imu error %lf %lf %lf\n", gyro_buf[0].header, cur_acc->header, gyro_buf[1].header);
             return;
         }
         
#ifdef READ_VINS
         if(start_playback_vins)
         {
             if(vinsDataFinished)
                 return;
             NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
             NSString *documentsPath = [paths objectAtIndex:0];
             NSString *filePath = [documentsPath stringByAppendingPathComponent:@"VINS"];
             vinsReader = [NSData dataWithContentsOfFile:filePath];
             
             [vinsReader getBytes:&vinsData range: NSMakeRange(vinsDataReadIndex * sizeof(vinsData), sizeof(vinsData))];
             vinsDataReadIndex++;
             if(vinsData.header == 0)
             {
                 printf("record play vins finished\n");
                 vinsDataFinished = true;
                 return;
             }
             printf("record play vins: %4d, %lf %lf %lf %lf %lf %lf %lf %lf\n",vinsDataReadIndex, vinsData.header, vinsData.translation.x(), vinsData.translation.y(), vinsData.translation.z(),
                    vinsData.rotation.w(), vinsData.rotation.x(), vinsData.rotation.y(),vinsData.rotation.z());
         }
#endif
         //for save data
         if(start_playback)
         {
             //TS(read_imu_buf);
             if(imuDataFinished)
                 return;
             [imuReader getBytes:&imuData range: NSMakeRange(imuDataReadIndex * sizeof(imuData), sizeof(imuData))];
             imuDataReadIndex++;
             if(imuData.header == 0)
             {
                 imuDataFinished = true;
                 return;
             }
             imu_msg->header = imuData.header;
             imu_msg->acc = imuData.acc;
             imu_msg->gyr = imuData.gyr;
             //TE(read_imu_buf);
#ifdef DATA_EXPORT
             printf("record play imu: %lf %lf %lf %lf %lf %lf %lf\n",imuData.header,imu_msg->acc.x(), imu_msg->acc.y(), imu_msg->acc.z(),
                    imu_msg->gyr.x(), imu_msg->gyr.y(), imu_msg->gyr.z());
#endif
         }
         
         if(start_record)
         {
             TS(record_imu_buf);
             imuData.header = imu_msg->header;
             imuData.acc = imu_msg->acc;
             imuData.gyr = imu_msg->gyr;
             [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
             imuDataIndex++;
             TE(record_imu_buf);
             //NSLog(@"record: imu %lf, %lu",imuData.header,imuDataIndex);
         }
         
         lateast_imu_time = imu_msg->header;
         
         //img_msg callback
         {
             IMU_MSG_LOCAL imu_msg_local;
             imu_msg_local.header = imu_msg->header;
             imu_msg_local.acc = imu_msg->acc;
             imu_msg_local.gyr = imu_msg->gyr;
             
             m_imu_feedback.lock();
             local_imu_msg_buf.push(imu_msg_local);
             m_imu_feedback.unlock();
         }
         m_buf.lock();
         imu_msg_buf.push(imu_msg);
         //NSLog(@"IMU_buf timestamp %lf, acc_x = %lf",imu_msg_buf.front()->header,imu_msg_buf.front()->acc.x());
         m_buf.unlock();
         con.notify_one();
     }];
}

/********************************************************************UI View Controler********************************************************************/
- (void)showInputView
{
    NSString *stringView;
    static bool finish_init = false;
    if(vins.solver_flag != vins.NON_LINEAR)
    {
        finish_init = false;
        switch (vins.init_status) {
            case vins.FAIL_IMU:
                stringView = [NSString stringWithFormat:@"STA: FAIL_IMU"];
                break;
            case vins.FAIL_PARALLAX:
                stringView = [NSString stringWithFormat:@"STA: FAIL_PARA"];
                break;
            case vins.FAIL_RELATIVE:
                stringView = [NSString stringWithFormat:@"STA: FAIL_RELA"];
                break;
            case vins.FAIL_SFM:
                stringView = [NSString stringWithFormat:@"STA: FAIL_SFM"];
                break;
            case vins.FAIL_PNP:
                stringView = [NSString stringWithFormat:@"STA: FAIL_PNP"];
                break;
            case vins.FAIL_ALIGN:
                stringView = [NSString stringWithFormat:@"STA: FAIL_ALIGN"];
                break;
            case vins.FAIL_CHECK:
                stringView = [NSString stringWithFormat:@"STA: FAIL_COST"];
                break;
            case vins.SUCC:
                stringView = [NSString stringWithFormat:@"STA: SUCC!"];
                break;
            default:
                break;
        }
        [_X_label setText:stringView];
        stringView = [NSString stringWithFormat:@"FAIL: %d times", vins.fail_times];
        [_Y_label setText:stringView];
        stringView = [NSString stringWithFormat:@"PARALLAX: %d", vins.parallax_num_view];
        [_Z_label setText:stringView];
        
        stringView = [NSString stringWithFormat:@"Initializing: %d%%", vins.initProgress];
        [_feature_label2 setText:stringView];
        
        [_feature_label2 setHidden:NO];
        [_feature_label3 setHidden:NO];
        [indicator setHidden:NO];
        [featureImageView setHidden:NO];
    }
    else
    {
        if(finish_init == false)
        {
            //Hide init UI
            [_feature_label2 setHidden:YES];
            [_feature_label3 setHidden:YES];
            [indicator setHidden:YES];
            [featureImageView setHidden:YES];
            
            start_show = true;
            finish_init = true;
        }
        
        float x_view = (float)vins.correct_Ps[frame_cnt][0];
        float y_view = (float)vins.correct_Ps[frame_cnt][1];
        float z_view = (float)vins.correct_Ps[frame_cnt][2];
        if(x_view_last == -5000)
        {
            x_view_last = x_view;
            y_view_last = y_view;
            z_view_last = z_view;
        }
        total_odom += sqrt(pow((x_view - x_view_last), 2) +
                           pow((y_view - y_view_last), 2) +
                           pow((z_view - z_view_last), 2));
        x_view_last = x_view;
        y_view_last = y_view;
        z_view_last = z_view;
        
        stringView = [NSString stringWithFormat:@"X:%.2f",x_view];
        [_X_label setText:stringView];
        stringView = [NSString stringWithFormat:@"TOTAL:%.2f",total_odom];
        //stringView = [NSString stringWithFormat:@"COST:%.2lf",vins.final_cost];
        //stringView = [NSString stringWithFormat:@"COST: %d, %.2lf",vins.visual_factor_num, vins.visual_cost];
        [_total_odom_label setText:stringView];
        stringView = [NSString stringWithFormat:@"Y:%.2f",y_view];
        [_Y_label setText:stringView];
        stringView = [NSString stringWithFormat:@"Z:%.2f",z_view];
        [_Z_label setText:stringView];
    }
    stringView = [NSString stringWithFormat:@"BUF:%d",waiting_lists];
    [_buf_label setText:stringView];
    //NSString *stringZ = [NSString stringWithFormat:@"Z:%.2f",z_view, vins.f_manager.getFeatureCount()];
    if(loop_old_index != -1)
    {
        stringView = [NSString stringWithFormat:@"LOOP with %d",loop_old_index];
        [_loop_label setText:stringView];
    }
    stringView = [NSString stringWithFormat:@"FEATURE: %d",vins.feature_num];
    [_feature_label setText:stringView];
}

-(void)showOutputImage:(UIImage*)image
{
    [featureImageView setImage:image];
}
/********************************************************************UI View Controler********************************************************************/


/********************************************************************UI Button Controler********************************************************************/

-(IBAction)switchUI:(UISegmentedControl *)sender
{
    switch (_switchUI.selectedSegmentIndex)
    {
        case 0:
            self.switchUIAREnabled = YES;
            
            printf("show AR\n");
            ui_main = true;
            box_in_AR= true;
            USE_PNP = true;
            imageCacheEnabled = cameraMode && !USE_PNP;
            break;
        case 1:
            self.switchUIAREnabled = NO;
            
            ui_main = false;
            if (box_in_AR)
                box_in_trajectory = true;
            USE_PNP = false;
            imageCacheEnabled = cameraMode && !USE_PNP;
            printf("show VINS\n");
            break;
        default:
            break;
    }
}

- (IBAction)fovSliderValueChanged:(id)sender {
    self.fovLabel.text = [[NSNumber numberWithFloat:self.fovSlider.value] stringValue];
    
}

- (void) handlePan:(UIPanGestureRecognizer*) recognizer
{
    if(ui_main and 0)
        return;
    
    if (!ui_main)
    {
        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_last = 0;
        static CGFloat vy_last = 0;
        
        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_last;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_last;
        vx_last = vx_smooth;
        vy_last = vy_smooth;
        if(recognizer.numberOfTouches == 2)
        {
            vins.drawresult.Y0 += vx_smooth/100.0;
            vins.drawresult.X0 += vy_smooth/100.0;
        }
        else
        {
            vins.drawresult.theta += vy_smooth/100.0;
            vins.drawresult.theta = fmod(vins.drawresult.theta, 360.0);
            vins.drawresult.phy += vx_smooth/100.0;
            vins.drawresult.phy = fmod(vins.drawresult.phy, 360.0);
        }
        
        vins.drawresult.change_view_manualy = true;
    }
    else
    {
        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //CGFloat translationX =
        //CGFloat translationY = [recognizer translationInView:self.view].y;
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!pipikk test x: %f y: %f", translationX, translationY);
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!%f  %f", imageView.frame.size.height, imageView.frame.size.width);
        CGPoint point = [recognizer locationInView:self.view];
        //NSLog(@"X Location: %f", point.x);
        //NSLog(@"Y Location: %f",point.y);
        
        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_lastAR = 0;
        static CGFloat vy_lastAR = 0;
        
        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_lastAR;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_lastAR;
        vx_lastAR = vx_smooth;
        vy_lastAR = vy_smooth;
        if(recognizer.numberOfTouches == 2)
        {
            
            vins.drawresult.Y0AR += vx_smooth/100.0;
            vins.drawresult.X0AR += vy_smooth/100.0;
            
            vins.drawresult.locationXT2 = point.x * 640.0 / imageView.frame.size.width;
            vins.drawresult.locationYT2 = point.y * 480.0 / imageView.frame.size.height;
            
            vins.drawresult.finger_s = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_d ++) > 7)
                vins.drawresult.finger_state = 2;
        }
        else
        {
            vins.drawresult.thetaAR += vy_smooth/100.0;
            //vins.drawresult.thetaAR = fmod(vins.drawresult.thetaAR, 360.0);
            vins.drawresult.phyAR += vx_smooth/100.0;
            //vins.drawresult.phyAR = fmod(vins.drawresult.phyAR, 360.0);
            
            vins.drawresult.locationX = point.x * 640.0 / imageView.frame.size.width;
            vins.drawresult.locationY = point.y * 480.0 / imageView.frame.size.height;
            
            vins.drawresult.finger_d = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_s ++) > 7)
                vins.drawresult.finger_state = 1;
        }
    }
    
    
}

- (void) handlePinch:(UIPinchGestureRecognizer*) recognizer
{
    if(ui_main and 0)
        return;
    
    if (!ui_main)
    {
        vins.drawresult.change_view_manualy = true;
        if(vins.drawresult.radius > 5 || recognizer.velocity < 0)
            vins.drawresult.radius -= recognizer.velocity * 0.5;
        else
        {
            vins.drawresult.Fx += recognizer.velocity * 15;
            if(vins.drawresult.Fx < 50)
                vins.drawresult.Fx = 50;
            vins.drawresult.Fy += recognizer.velocity * 15;
            if(vins.drawresult.Fy < 50)
                vins.drawresult.Fy = 50;
        }
    }
    else{
        
        vins.drawresult.finger_s = 0;
        vins.drawresult.finger_d = 0;
        if ((vins.drawresult.finger_p ++) > 7)
            vins.drawresult.finger_state = 3;
        
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationXP = point.x * 640.0 / imageView.frame.size.width;
        vins.drawresult.locationYP = point.y * 480.0 / imageView.frame.size.height;
        
        //NSLog(@"pipikk_radius: %f velocity: ", vins.drawresult.radiusAR, recognizer.velocity);
        
        //if(vins.drawresult.radiusAR > 5 || recognizer.velocity < 0)
        //{
        vins.drawresult.radiusAR -= recognizer.velocity * 0.5;
        //}
    }
    
}

- (void) handleTap:(UITapGestureRecognizer*) recognizer
{
    if (!ui_main)
    {
        
    }
    else{
        
        /*vins.drawresult.finger_s = 0;
         vins.drawresult.finger_d = 0;
         if ((vins.drawresult.finger_p ++) > 7)
         vins.drawresult.finger_state = 3;*/
        
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationTapX = point.x * 640.0 / imageView.frame.size.width;
        vins.drawresult.locationTapY = point.y * 480.0 / imageView.frame.size.height;
        
        vins.drawresult.tapFlag = true;
        
    }
    
}

- (void) handleLongPress:(UILongPressGestureRecognizer*) recognizer
{
    if (!ui_main)
    {
        
    }
    {
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationLongPressX = point.x * 640.0 / imageView.frame.size.width;
        vins.drawresult.locationLongPressY = point.y * 480.0 / imageView.frame.size.height;
        vins.drawresult.longPressFlag = true;
    }
}

- (IBAction)loopButtonPressed:(id)sender {
    if(LOOP_CLOSURE)
    {
        LOOP_CLOSURE = false;
        [_loopButton setTitle:@"ENLOOP" forState:UIControlStateNormal];
    }
    else
    {
        LOOP_CLOSURE = true;
        [_loopButton setTitle:@"UNLOOP" forState:UIControlStateNormal];
    }
    /*
     start_record = !start_record;
     if(start_record)
     {
     start_playback = false;
     [_recordButton setTitle:@"Stop" forState:UIControlStateNormal];
     [saveData start];
     }
     else
     {
     TS(record_imu);
     imuData.header = 0; // as the ending marker
     imuData.acc << 0,0,0;
     imuData.gyr << 0,0,0;
     [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
     [self recordImu];
     TE(record_imu);
     [_recordButton setTitle:@"Record" forState:UIControlStateNormal];
     }
     */
}

- (IBAction)reinitButtonPressed:(id)sender {
    vins.drawresult.planeInit = false;
    vins.failure_hand = true;
    vins.drawresult.change_color = true;
    vins.drawresult.indexs.push_back(vins.drawresult.pose.size());
    segmentation_index++;
    keyframe_database.max_seg_index++;
    keyframe_database.cur_seg_index = keyframe_database.max_seg_index;
    /*
     start_playback = !start_playback;
     if(start_playback)
     {
     //TS(read_imu);
     NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
     NSString *documentsPath = [paths objectAtIndex:0];
     NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name
     imuReader = [NSData dataWithContentsOfFile:filePath];
     //TE(read_imu);
     start_record = false;
     [_playbackButton setTitle:@"Stop" forState:UIControlStateNormal];
     }
     else
     [_playbackButton setTitle:@"Playback" forState:UIControlStateNormal];
     */
}

/********************************************************************UI Button Controler********************************************************************/


/***********************************************************About record and playback data for debug********************************************************/

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

-(void)saveData{
    while (![[NSThread currentThread] isCancelled])
    {
        @autoreleasepool
        {
            if(!imgDataBuf.empty())
            {
                IMG_DATA tmp_data;
                tmp_data = imgDataBuf.front();
                imgDataBuf.pop();
                [self recordImageTime:tmp_data];
                [self recordImage:tmp_data];
                imageDataIndex++;
                //NSLog(@"record: %lf %lu",tmp_data.header,imageDataIndex);
            }
        }
        [NSThread sleepForTimeInterval:0.04];
    }
}

- (void)tapSaveImageToIphone:(UIImage*)image
{
    UIImageWriteToSavedPhotosAlbum(image, self, @selector(image:didFinishSavingWithError:contextInfo:), nil);
}

- (void)image:(UIImage *)image didFinishSavingWithError:(NSError *)error contextInfo:(void *)contextInfo{
    
    if (error == nil) {
        NSLog(@"save access");
    }else{
        NSLog(@"save failed");
    }
}

- (void)checkDirectoryPath:(unsigned long)index withObject:(NSString*)directoryPath
{
    //delete already exist directory first time
    NSError *error;
    if (index == 0 && [[NSFileManager defaultManager] fileExistsAtPath:directoryPath])	//Does directory exist?
    {
        if (![[NSFileManager defaultManager] removeItemAtPath:directoryPath error:&error])	//Delete it
        {
            NSLog(@"Delete directory error: %@", error);
        }
    }
    
    //creat file directory if it does not exist
    if (![[NSFileManager defaultManager] fileExistsAtPath:directoryPath])
    {
        NSLog(@"directory does not exist");
        if (![[NSFileManager defaultManager] createDirectoryAtPath:directoryPath
                                       withIntermediateDirectories:NO
                                                        attributes:nil
                                                             error:&error])
        {
            NSLog(@"Create directory error: %@", error);
        }
    }
}

- (void)recordImu
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name
    
    [imuDataBuf writeToFile:filePath atomically:YES];
    //[msgData writeToFile:filePath atomically:YES];
}

- (void)recordVins
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"VINS"]; //Add the file name
    
    [vinsDataBuf writeToFile:filePath atomically:YES];
    //[msgData writeToFile:filePath atomically:YES];
}

- (void)recordImageTime:(IMG_DATA&)image_data
{
    double time = image_data.header;
    NSData *msgData = [NSData dataWithBytes:&time length:sizeof(time)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"];; //Get the docs directory
    
    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];
    
    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    [msgData writeToFile:filePath atomically:YES];
}

- (void)recordImage:(IMG_DATA&)image_data
{
    NSData *msgData = UIImagePNGRepresentation(image_data.image);
    //NSData *msgData = [NSData dataWithBytes:&image_data length:sizeof(image_data)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"];; //Get the docs directory
    
    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];
    
    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    [msgData writeToFile:filePath atomically:YES];
}

-(bool)readImageTime:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *file1 = [[NSData alloc] initWithContentsOfFile:filePath];
        if (file1)
        {
            double time;
            [file1 getBytes:&time length:sizeof(time)];
            imgData.header = time;
        }
        file_exist = true;
    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}

-(bool)readImage:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *pngData = [NSData dataWithContentsOfFile:filePath];
        imgData.image = [UIImage imageWithData:pngData];
        file_exist = true;
    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}

/**************************************************************About record and playback data for debug**********************************************************/

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
}

- (void)viewDidDisappear:(BOOL)animated
{
    [super viewDidDisappear:animated];
    if (isCapturing)
    {
        [videoCamera stop];
    }
    [mainLoop cancel];
    [draw cancel];
#ifdef LOOP_CLOSURE
    [loop_thread cancel];
#endif
    
}

-(void)viewDidUnload{
    [motionManager stopAccelerometerUpdates];
    [motionManager stopDeviceMotionUpdates];
    [motionManager stopGyroUpdates];
    [motionManager stopMagnetometerUpdates];
    [super viewDidUnload];
}

- (void)dealloc
{
    videoCamera.delegate = nil;
}

/*
 Check the device
 */
DeviceType deviceName()
{
    struct utsname systemInfo;
    uname(&systemInfo);
    
    NSString *device = [NSString stringWithCString:systemInfo.machine
                                          encoding:NSUTF8StringEncoding];
    DeviceType device_type;
    if(([device compare:@"iPhone9,1"] == NSOrderedSame) ||
       ([device compare:@"iPhone9,3"] == NSOrderedSame))
    {
        printf("Device iPhone7\n");
        device_type = iPhone7;
    }
    else if(([device compare:@"iPhone9,2"] == NSOrderedSame) ||
            ([device compare:@"iPhone9,4"] == NSOrderedSame))
    {
        printf("Device iPhone7 plus\n");
        device_type = iPhone7P;
    }
    else if(([device compare:@"iPhone8,2"] == NSOrderedSame))
    {
        printf("Device iPhone6s plus\n");
        device_type = iPhone6sP;
    }
    else if(([device compare:@"iPhone8,1"] == NSOrderedSame))
    {
        printf("Device iPhone6s\n");
        device_type = iPhone6s;
    }
    else if(([device compare:@"iPad6,3"] == NSOrderedSame)||
            ([device compare:@"iPad6,4"] == NSOrderedSame))
    {
        printf("Device iPad pro 9.7\n");
        device_type = iPadPro97;
    }
    else if(([device compare:@"iPad6,7"] == NSOrderedSame)||
            ([device compare:@"iPad6,8"] == NSOrderedSame))
    {
        printf("Device iPad pro 12.9\n");
        device_type = iPadPro129;
    }
    else
    {
        printf("Device undefine\n");
        device_type = unDefine;
    }
    return device_type;
}

bool iosVersion()
{
    NSComparisonResult order = [[UIDevice currentDevice].systemVersion compare: @"10.2.1" options: NSNumericSearch];
    if (order == NSOrderedSame || order == NSOrderedDescending) {
        printf("system version >= 10.2.1\n");
        return true;
    } else {
        printf("system version < 10.2.1\n");
        return false;
    }
}
@end

#ifndef __KEY_FRAME_
#define __KEY_FRAME_


#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "utility.hpp"
#include <algorithm>
#include "math.h"
#include "global_param.hpp"
#include "VINS.hpp"
#include "loop_closure.h"

using namespace Eigen;
using namespace std;
//using namespace cv;

class BriefExtractor: public FeatureExtractor<FBrief::TDescriptor>
{
public:
    virtual void operator()(const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
                            vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;
    BriefExtractor(const std::string &pattern_file);
    
private:
    DVision::BRIEF m_brief;
};

struct matchCluster
{
    //stores nearest points with old vins point
    std::vector<int> indexs; //bow points index
    int best_index;
};

class KeyFrame
{
public:
    KeyFrame(double _header, int _global_index, Eigen::Vector3d _T_w_c, Eigen::Matrix3d _R_w_c, cv::Mat &_image, const char *_brief_pattern_file, const int _segment_index);
    void setExtrinsic(Eigen::Vector3d T, Eigen::Matrix3d R);
    void initPtsByReprojection(Eigen::Vector3d Ti_predict,
                               Eigen::Matrix3d Ri_predict,
                               std::vector<cv::Point2f> &measurements_predict);
    void initPoseForPnP(Eigen::Vector3d &T_c_w,
                        Eigen::Matrix3d &R_c_w);
    void cam2Imu(Eigen::Vector3d T_c_w,
                 Eigen::Matrix3d R_c_w,
                 Eigen::Vector3d &T_w_i,
                 Eigen::Matrix3d &R_w_i);
    void rejectWithF(vector<cv::Point2f> &measurements_old,
                     vector<cv::Point2f> &measurements_old_norm);
    void extractBrief(cv::Mat &image);
    void searchInBoW(std::vector<cv::Point2f> &cur_pts,
                     std::vector<cv::Point2f> &old_pts,
                     std::vector<cv::Point2f> &old_measurements);
    void buildKeyFrameFeatures(VINS &vins);
    
    void searchByDes(std::vector<cv::Point2f> &measurements_old,
                     std::vector<cv::Point2f> &measurements_old_norm,
                     const std::vector<BRIEF::bitset> &descriptors_old,
                     const std::vector<cv::KeyPoint> &keypoints_old);
    
    bool solveOldPoseByPnP(std::vector<cv::Point2f> &measurements_old_norm,
                           const Eigen::Vector3d T_w_i_old, const Eigen::Matrix3d R_w_i_old,
                           Eigen::Vector3d &T_w_i_refine, Eigen::Matrix3d &R_w_i_refine);
    
    bool findConnectionWithOldFrame(const KeyFrame* old_kf,
                                    const std::vector<cv::Point2f> &cur_pts, const std::vector<cv::Point2f> &old_pts,
                                    std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm);
    void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
    
    void updateOriginPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
    
    void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
    
    void getOriginPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
    
    void addConnection(int index, KeyFrame* connected_kf);
    
    void addConnection(int index, KeyFrame* connected_kf, Vector3d relative_t, Quaterniond relative_q, double relative_yaw);
    
    void addLoopConnection(int index, KeyFrame* loop_kf);
    
    void updateLoopConnection(Vector3d relative_t, Quaterniond relative_q, double relative_yaw);
    
    void detectLoop(int index);
    
    void removeLoop();
    
    int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);
    
    // data
    double header;
    std::vector<Eigen::Vector3d> point_clouds, point_clouds_origin;
    //feature in origin image plane
    std::vector<cv::Point2f> measurements, measurements_origin;
    //feature in normalize image plane
    std::vector<cv::Point2f> pts_normalize;
    //feature ID
    std::vector<int> features_id, features_id_origin;
    //feature descriptor
    std::vector<BRIEF::bitset> descriptors;
    //keypoints
    std::vector<cv::KeyPoint> keypoints;
    
    int global_index;
    cv::Mat image;
    Matrix3d qic;
    Vector3d tic;
    bool use_retrive;
    bool has_loop;
    bool check_loop;
    // looped by other frame
    bool is_looped;
    int loop_index;
    int resample_index;
    const char *BRIEF_PATTERN_FILE;
    // index t_w t_y t_z q_w q_x q_y q_z yaw
    list<pair<int, Eigen::Matrix<double, 8, 1 > > > connection_list;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int segment_index;
    
private:
    Eigen::Vector3d T_w_i;
    Eigen::Matrix3d R_w_i;
    Eigen::Vector3d origin_T_w_i;
    Eigen::Matrix3d origin_R_w_i;
    std::mutex mMutexPose;
    std::vector<cv::KeyPoint> window_keypoints;
    std::vector<BRIEF::bitset> window_descriptors;
    
};

#endif


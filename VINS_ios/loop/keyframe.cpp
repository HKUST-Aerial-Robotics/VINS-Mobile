#include "keyframe.h"


KeyFrame::KeyFrame(double _header, int _global_index, Eigen::Vector3d _T_w_i, Eigen::Matrix3d _R_w_i,
                   cv::Mat &_image, const char *_brief_pattern_file, const int _segment_index)
:header{_header}, global_index{_global_index}, T_w_i{_T_w_i}, R_w_i{_R_w_i}, image{_image}, BRIEF_PATTERN_FILE(_brief_pattern_file), segment_index(_segment_index)
{
    use_retrive = 0;
    is_looped = 0;
    has_loop = 0;
    origin_T_w_i = _T_w_i;
    origin_R_w_i = _R_w_i;
    check_loop = 0;
}

/*****************************************utility function************************************************/
bool inBorder2(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

template <typename T>
void reduceVector2(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void KeyFrame::rejectWithF(vector<cv::Point2f> &measurements_old,
                 vector<cv::Point2f> &measurements_old_norm)
{
    if (measurements_old.size() >= 8)
    {
        measurements_old_norm.clear();

        for (unsigned int i = 0; i < measurements_old.size(); i++)
        {
            cv::Point2f norm_pt;
            norm_pt.x = (measurements_old[i].x - PX)/FOCUS_LENGTH_X;
            norm_pt.y = (measurements_old[i].y - PY)/FOCUS_LENGTH_Y;
            measurements_old_norm.push_back(norm_pt);
        }

        vector<uchar> status;
        cv::findFundamentalMat(measurements, measurements_old, cv::FM_RANSAC, 2.0, 0.99, status);
        reduceVector2(point_clouds, status);
        reduceVector2(measurements, status);
        reduceVector2(measurements_old, status);
        reduceVector2(measurements_old_norm, status);
        reduceVector2(features_id, status);
    }
}
/*****************************************utility function************************************************/

void KeyFrame::extractBrief(cv::Mat &image)
{
    BriefExtractor extractor(BRIEF_PATTERN_FILE);
    extractor(image, measurements, keypoints, descriptors);
    int start = keypoints.size() - measurements.size();
    for(int i = 0; i< measurements.size(); i++)
    {
        window_keypoints.push_back(keypoints[start + i]);
        window_descriptors.push_back(descriptors[start + i]);
    }
}

void KeyFrame::setExtrinsic(Eigen::Vector3d T, Eigen::Matrix3d R)
{
    qic = R;
    tic = T;
}

void KeyFrame::initPtsByReprojection(Eigen::Vector3d Ti_predict,
									 Eigen::Matrix3d Ri_predict,
									 std::vector<cv::Point2f> &measurements_predict)
{
    measurements_predict.clear();
    Vector3d pts_predict;
	for(int i = 0; i < (int)point_clouds.size(); i++)
	{
    	Eigen::Vector3d pts_w = point_clouds[i];
    	Eigen::Vector3d pts_imu_j = Ri_predict.inverse() * (pts_w - Ti_predict);
    	Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    	pts_predict <<  pts_camera_j.x()/pts_camera_j.z(),
    				    pts_camera_j.y()/pts_camera_j.z(),
    					1.0;
        Vector2d point_uv;
        point_uv.x() = FOCUS_LENGTH_X * pts_predict.x() + PX;
        point_uv.y() = FOCUS_LENGTH_Y * pts_predict.y() + PY;
        measurements_predict.push_back(cv::Point2f(point_uv.x(), point_uv.y()));
	}
    if(measurements_predict.size() == 0)
    {
        measurements_predict = measurements;
    }
}

void KeyFrame::initPoseForPnP(Eigen::Vector3d &T_c_w,
							  Eigen::Matrix3d &R_c_w)
{
	Matrix3d R_w_c = R_w_i * qic;
    Vector3d T_w_c = T_w_i + R_w_i * tic;

    R_c_w = R_w_c.inverse();
    T_c_w = -(R_c_w * T_w_c);
}

void KeyFrame::cam2Imu(Eigen::Vector3d T_c_w,
                       Eigen::Matrix3d R_c_w,
                       Eigen::Vector3d &t_w_i,
                       Eigen::Matrix3d &r_w_i)
{
    r_w_i = (qic * R_c_w).inverse();
    t_w_i = -R_c_w.inverse() * T_c_w - r_w_i * tic;
}

double round(double r)  
{  
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);  
} 

void KeyFrame::buildKeyFrameFeatures(VINS &vins)
{
    for (auto &it_per_id : vins.f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        int frame_size = it_per_id.feature_per_frame.size();
        if(it_per_id.start_frame + frame_size >= WINDOW_SIZE - 1&& frame_size >=3)
        {
            //features current measurements
            Vector3d point = it_per_id.feature_per_frame[WINDOW_SIZE - 2 - it_per_id.start_frame].point;
            Vector2d point_uv;
            point_uv.x() = FOCUS_LENGTH_X * point.x()/point.z() + PX;
            point_uv.y() = FOCUS_LENGTH_Y * point.y()/point.z() + PY;
            measurements.push_back(cv::Point2f(point_uv.x(), point_uv.y()));
            pts_normalize.push_back(cv::Point2f(point.x()/point.z(), point.y()/point.z()));
            
            features_id.push_back(it_per_id.feature_id);
            //features 3D pos from first measurement and inverse depth
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            point_clouds.push_back(vins.Rs[it_per_id.start_frame] * (vins.ric * pts_i + vins.tic) + vins.Ps[it_per_id.start_frame]);
        }
    }
    measurements_origin  = measurements;
    point_clouds_origin = point_clouds;
    features_id_origin = features_id;
}

/**
 ** search matches by guide descriptor match
 **
 **/
void KeyFrame::searchByDes(std::vector<cv::Point2f> &measurements_old,
                           std::vector<cv::Point2f> &measurements_old_norm,
                           const std::vector<BRIEF::bitset> &descriptors_old,
                           const std::vector<cv::KeyPoint> &keypoints_old)
{
    printf("loop_match before cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
    std::vector<int> dis_cur_old;
    std::vector<uchar> status;
    for(int i = 0; i < window_descriptors.size(); i++)
    {
        int bestDist = 256;
        int bestIndex = -1;
        for(int j = 0; j < descriptors_old.size(); j++)
        {
            int dis = HammingDis(window_descriptors[i], descriptors_old[j]);
            if(dis < bestDist)
            {
                bestDist = dis;
                bestIndex = j;
            }
        }
        if(bestDist < 256)
        {
            measurements_old.push_back(keypoints_old[bestIndex].pt);
            dis_cur_old.push_back(bestDist);
        }
    }
    rejectWithF(measurements_old, measurements_old_norm);
    printf("loop_match after cur %d %d, old %d\n", window_descriptors.size(), measurements.size(), descriptors_old.size());
}

/**
*** return refined pose of the current frame
**/
bool KeyFrame::solveOldPoseByPnP(std::vector<cv::Point2f> &measurements_old_norm, 
                                 const Eigen::Vector3d T_w_i_old, const Eigen::Matrix3d R_w_i_old,
                                 Eigen::Vector3d &T_w_i_refine, Eigen::Matrix3d &R_w_i_refine)
{
    //solve PnP get pose refine
    cv::Mat r, rvec, t, D, tmp_r;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    Matrix3d R_inital;
    Vector3d P_inital;
    Matrix3d R_w_c = R_w_i_old * qic;
    Vector3d T_w_c = T_w_i_old + R_w_i * tic;

    R_inital = R_w_c.inverse();
    P_inital = -(R_inital * T_w_c);
    
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    vector<cv::Point3f> pts_3_vector;
    bool pnp_succ = false;
    for(auto &it: point_clouds)
        pts_3_vector.push_back(cv::Point3f((float)it.x(),(float)it.y(),(float)it.z()));
    if(pts_3_vector.size()>=30)
    {
        if(!use_retrive)
        {
            pnp_succ = cv::solvePnP(pts_3_vector, measurements_old_norm, K, D, rvec, t, 1);
        }
        else
        {
            initPoseForPnP(P_inital,R_inital);
            cv::eigen2cv(R_inital, tmp_r);
            cv::Rodrigues(tmp_r, rvec);
            cv::eigen2cv(P_inital, t);
            pnp_succ = cv::solvePnP(pts_3_vector, measurements_old_norm, K, D, rvec, t, 1);
        }
    }

    if(!pnp_succ)
    {
        cout << "loop pnp failed !" << endl;
        return false;
    }
    else
    {
        cout << "loop pnp succ !" << endl;
    }
    cv::Rodrigues(rvec, r);
    Matrix3d R_loop;
    cv::cv2eigen(r, R_loop);
    Vector3d T_loop;
    cv::cv2eigen(t, T_loop);
    
    Vector3d old_T_drift;
    Matrix3d old_R_drift;
    cam2Imu(T_loop, R_loop, old_T_drift, old_R_drift);

    T_w_i_refine = T_w_i_old + R_w_i_old * old_R_drift.transpose() * (T_w_i - old_T_drift);
    R_w_i_refine = R_w_i_old * old_R_drift.transpose() * R_w_i;

    //printf("loop current T: %2lf %2lf %2lf\n", T_w_i(0),T_w_i(1),T_w_i(2));
                
    //printf("loop refined T: %2lf %2lf %2lf\n", T_w_i_refine(0),T_w_i_refine(1),T_w_i_refine(2));
    return true;
}

/**
*** interface to VINS
*** input: looped old keyframe which include image and pose, and feature correnspondance given by BoW
*** output: ordered old feature correspondance with current KeyFrame and the translation drift
**/
bool KeyFrame::findConnectionWithOldFrame(const KeyFrame* old_kf,
                                          const std::vector<cv::Point2f> &cur_pts, const std::vector<cv::Point2f> &old_pts,
                                          std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm)
{
    searchByDes(measurements_old, measurements_old_norm, old_kf->descriptors, old_kf->keypoints);
    return true;
}

void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    T_w_i = _T_w_i;
    R_w_i = _R_w_i;
}

void KeyFrame::updateOriginPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    origin_T_w_i = _T_w_i;
    origin_R_w_i = _R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = T_w_i;
    _R_w_i = R_w_i;
}

void KeyFrame::getOriginPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = origin_T_w_i;
    _R_w_i = origin_R_w_i;
}

void KeyFrame::addConnection(int index, KeyFrame* connected_kf)
{
    Vector3d connected_t, relative_t;
    Matrix3d connected_r;
    Quaterniond relative_q;
    connected_kf->getPose(connected_t, connected_r);
    
    relative_q = connected_r.transpose() * R_w_i;
    relative_t = connected_r.transpose() * (T_w_i - connected_t);
    double relative_yaw;
    relative_yaw = Utility::R2ypr(R_w_i).x() - Utility::R2ypr(connected_r).x();
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    connection_list.push_back(make_pair(index, connected_info));
}

void KeyFrame::addConnection(int index, KeyFrame* connected_kf, Vector3d relative_t, Quaterniond relative_q, double relative_yaw)
{
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    connection_list.push_back(make_pair(index, connected_info));
}

void KeyFrame::addLoopConnection(int index, KeyFrame* loop_kf)
{
    assert(index == loop_index);
    Vector3d connected_t, relative_t;
    Matrix3d connected_r;
    Quaterniond relative_q;
    loop_kf->getPose(connected_t, connected_r);
    
    relative_q = connected_r.transpose() * R_w_i;
    relative_t = connected_r.transpose() * (T_w_i - connected_t);
    double relative_yaw;
    relative_yaw = Utility::R2ypr(R_w_i).x() - Utility::R2ypr(connected_r).x();
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    loop_info = connected_info;
}

void KeyFrame::updateLoopConnection(Vector3d relative_t, Quaterniond relative_q, double relative_yaw)
{
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    relative_yaw;
    loop_info = connected_info;
}

void KeyFrame::detectLoop(int index)
{
    has_loop = true;
    loop_index = index;
}

int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
    // The DVision::BRIEF extractor computes a random pattern by default when
    // the object is created.
    // We load the pattern that we used to build the vocabulary, to make
    // the descriptors compatible with the predefined vocabulary
    
    // loads the pattern
    cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
    if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;
    
    vector<int> x1, y1, x2, y2;
    fs["x1"] >> x1;
    fs["x2"] >> x2;
    fs["y1"] >> y1;
    fs["y2"] >> y2;
    
    m_brief.importPairs(x1, y1, x2, y2);
}

void BriefExtractor::operator() (const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
                                 vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
    // extract FAST keypoints with opencv
    const int fast_th = 20; // corner detector response threshold
    cv::FAST(im, keys, fast_th, true);
    for(int i = 0; i < window_pts.size(); i++)
    {
        cv::KeyPoint key;
        key.pt = window_pts[i];
        keys.push_back(key);
    }
    // compute their BRIEF descriptor
    m_brief.compute(im, keys, descriptors);
}

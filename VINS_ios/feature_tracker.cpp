//
//  feature_tracker.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "feature_tracker.hpp"

int FeatureTracker::n_id = 0;
FeatureTracker::FeatureTracker()
:mask{ROW, COL, CV_8UC1},update_finished{false},img_cnt{0},current_time{-1.0},use_pnp{false}
{
    printf("init ok\n");
}
/*********************************************************tools function for feature tracker start*****************************************************/
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

template <typename T>
void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
        max_min_pts tmp;
        tmp.min = p;
        tmp.max = p;
        parallax_cnt.push_back(tmp);
    }
}

void FeatureTracker::setMask()
{
    mask.setTo(255);
    
    // prefer to keep features that are tracked for long time
    
    vector<pair<pair<int, max_min_pts>, pair<cv::Point2f, int>>> cnt_pts_id;
    
    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(make_pair(track_cnt[i], parallax_cnt[i]), make_pair(forw_pts[i], ids[i])));
    
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &a, const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &b)
         {
             return a.first.first > b.first.first;
         });
    
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    parallax_cnt.clear();
    
    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
            //if(true)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first.first);
            parallax_cnt.push_back(it.first.second);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
    //for (auto &it: pre_pts)
    //{
    //    cv::circle(mask, it, MIN_DIST, 0, -1);
    //}
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        vector<uchar> status;
        
        cv::findFundamentalMat(pre_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        reduceVector(pre_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(parallax_cnt, status);
    }
}

/*********************************************************tools function for feature tracker ending*****************************************************/

bool FeatureTracker::solveVinsPnP(double header, Vector3d &P, Matrix3d &R, bool vins_normal)
{
    if(!vins_normal)
        return false;
    /*
     if(solved_features.size() < 2)
     {
     printf("pnp not enough features\n");
     return false;
     }
     */
    vector<IMG_MSG_LOCAL> feature_msg;
    int i = 0;
    for (auto &it : solved_features)
    {
        while(ids[i] < it.id)
        {
            i++;
        }
        if(ids[i] == it.id)
        {
            IMG_MSG_LOCAL tmp;
            tmp = it;
            tmp.observation = (Vector2d((forw_pts[i].x - PX)/FOCUS_LENGTH_X, (forw_pts[i].y - PY)/FOCUS_LENGTH_Y));
            feature_msg.push_back(tmp);
        }
    }
    /*
     if(feature_msg.size() < 2 )
     {
     printf("pnp Not enough solved feature!\n");
     return false;
     }
     */
    vins_pnp.setInit(solved_vins);
    printf("pnp imu header: ");
    for(auto &it : imu_msgs)
    {
        double t = it.header;
        if (current_time < 0)
            current_time = t;
        double dt = (t - current_time);
        current_time = t;
        printf("%lf ",t);
        vins_pnp.processIMU(dt, it.acc, it.gyr);
    }
    printf("image %lf\n", header);
    vins_pnp.processImage(feature_msg, header, use_pnp);
    
    P = vins_pnp.Ps[PNP_SIZE - 1];
    R = vins_pnp.Rs[PNP_SIZE - 1];
    Vector3d R_ypr = Utility::R2ypr(R);
    return true;
}

void FeatureTracker::readImage(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<Point2f> &good_pts, vector<double> &track_len, double header, Vector3d &P, Matrix3d &R, bool vins_normal)
{
    
    result = _img;
    if(forw_img.empty())
        pre_img = cur_img = forw_img = _img;
    else
        forw_img = _img;
    
    forw_pts.clear();
    
    //track
    {
        if(cur_pts.size()>0)
        {
            vector<uchar> status;
            vector<float> err;
            
            //TS(time_track);
            calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
            //TE(time_track);
            for (int i = 0; i < int(forw_pts.size()); i++)
                if (status[i] && !inBorder(forw_pts[i]))
                    status[i] = 0;
            reduceVector(pre_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(forw_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(parallax_cnt, status);
            
            //reject outliers
            if (forw_pts.size() >= 8)
            {
                vector<uchar> status;
                
                cv::findFundamentalMat(cur_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                reduceVector(cur_pts, status);
                reduceVector(pre_pts, status);
                reduceVector(forw_pts, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                reduceVector(parallax_cnt, status);
            }
            
            solveVinsPnP(header, P, R, vins_normal);
            
            if(img_cnt!=0)
            {
                for (int i = 0; i< forw_pts.size(); i++)
                {
                    //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                    good_pts.push_back(forw_pts[i]);
                    if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = forw_pts[i];
                    }
                    else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = forw_pts[i];
                    }
                    double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                    track_len.push_back(std::min(1.0, 1.0 * parallax/30));
                }
            }
        }
    }
    
    //detect
    {
        
        if(img_cnt==0)
        {
            rejectWithF();
            
            for (int i = 0; i< forw_pts.size(); i++)
            {
                good_pts.push_back(forw_pts[i]);
                if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                {
                    parallax_cnt[i].min = forw_pts[i];
                }
                else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                {
                    parallax_cnt[i].max = forw_pts[i];
                }
                double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                track_len.push_back(std::min(1.0, 1.0 * parallax/50));
            }
            
            for (auto &n : track_cnt)
                n++;
            
            setMask();
            int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
            
            if(n_max_cnt>0)
            {
                n_pts.clear();
                TS(time_goodfeature);
                //goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, MIN_DIST, mask, 3, false, 0.04);
                goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
                TE(time_goodfeature);
            }
            else
            {
                n_pts.clear();
            }
            
            addPoints();
            //printf("features num after detect: %d\n",static_cast<int>(forw_pts.size()));
            pre_img = forw_img;
            pre_pts = forw_pts;
            //draw
            for (int i = 0; i < n_pts.size(); i++)
            {
                good_pts.push_back(n_pts[i]);
                track_len.push_back(0);
            }
            //result = mask;
            
        }
        cur_img = forw_img;
        cur_pts = forw_pts;
    }
    if(img_cnt == 0)
    {
        //update id and msg
        image_msg.clear();
        int num_new = 0;
        
        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            completed |= updateID(i);
            if (!completed)
                break;
        }
        for(int i = 0; i<ids.size(); i++)
        {
            double x = (cur_pts[i].x - PX)/FOCUS_LENGTH_X;
            double y = (cur_pts[i].y - PY)/FOCUS_LENGTH_Y;
            double z = 1.0;
            image_msg[(ids[i])] = (Vector3d(x, y, z));
        }
    }
    //finished and tell solver the data is ok
    update_finished = true;
}
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

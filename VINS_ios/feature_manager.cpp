//
//  feature_manager.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/20.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "feature_manager.hpp"

int FeaturePerId::endFrame()
{
    return (int)(start_frame + feature_per_frame.size() - 1);
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
: Rs(_Rs)
{
    ric = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r));
}

double FeatureManager::compensatedParallax1(FeaturePerId &f_per_id)
{
    int l = f_per_id.feature_per_frame.size();
    FeaturePerFrame &frame_i = f_per_id.feature_per_frame[0];
    FeaturePerFrame &frame_j = f_per_id.feature_per_frame[l - 1];
    
    int r_i = f_per_id.start_frame + 0;
    int r_j = f_per_id.start_frame + l - 1;
    
    Vector3d p_i = frame_i.point;
    
    double u_i = p_i(0);
    double v_i = p_i(1);
    
    double ans = 0;
    
    Vector3d p_j = frame_j.point;
    Vector3d p_j_comp;
    p_j_comp = ric.transpose() * Rs[r_i].transpose() * Rs[r_j] * ric * p_j;
    
    double dep_j = p_j(2);
    double u_j = p_j(0) / dep_j;
    double v_j = p_j(1) / dep_j;
    
    double du = u_i - u_j, dv = v_i - v_j;
    double dep_j_comp = p_j_comp(2);
    double u_j_comp = p_j_comp(0) / dep_j_comp;
    double v_j_comp = p_j_comp(1) / dep_j_comp;
    double du_comp = u_i - u_j_comp, dv_comp = v_i - v_j_comp;
    
    double para = sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp));
    
    frame_j.parallax = para;
    
    if (r_i == r_j) //the feature appeared first time
    {
        para = 1e-3;
    }
    ans = max(ans, para);
    
    return ans;
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];
    
    int r_i = frame_count - 2;
    int r_j = frame_count - 1;
    
    double ans = 0;
    Vector3d p_j = frame_j.point;
    
    double u_j = p_j(0);
    double v_j = p_j(1);
    
    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp = p_i;
    //p_i_comp = ric.transpose() * Rs[r_j].transpose() * Rs[r_i] * ric * p_i;
    
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    
    double du = u_i - u_j, dv = v_i - v_j;
    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;
    
    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));
    return ans;
}

/*
 Check if the current frame has enough parallax compare with previous frame
 if have, return true;
 if no, return false;
 At the sametime, add the new feature observation to the feature class
 */
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, Vector3d> &image_msg, int &parallax_num)
{
    double parallax_sum = 0;
    parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image_msg)
    {
        FeaturePerFrame f_per_fra(id_pts.second);
        
        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
                              return it.feature_id == feature_id;
                          });
        //new feature
        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count)); //give id and start frame
            feature.back().feature_per_frame.push_back(f_per_fra);    //give point
        }
        //find match with previous feature
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num ++;
        }
    }
    
    if (frame_count < 2 || last_track_num < 20)
        return true;
    
    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }
    
    printf("parallax sum = %lf parallax_num = %d\n",parallax_sum, parallax_num);
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        //ROS_INFO("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        //ROS_INFO("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;
            
            a = it.feature_per_frame[idx_l].point;
            
            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic, Matrix3d ric, bool is_nonlinear)
{
    outlier_info.clear();
    for (auto &it_per_id : feature)
    {
        if(it_per_id.feature_per_frame.size()>= WINDOW_SIZE)
        {
            it_per_id.fixed = true;
            //cout << "track num" << it_per_id->feature_per_frame.size() << endl;
        }
        
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        
        if (it_per_id.estimated_depth > 0)
            continue;
        
        it_per_id.is_outlier = false;
        
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d p_i = Ps[imu_i], p_j;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point, pts_j;
        
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;
        
        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic;
        Eigen::Matrix3d R0 = Rs[imu_i] * ric;
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();
        
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic;
            Eigen::Matrix3d R1 = Rs[imu_j] * ric;
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);
            
            if (imu_i == imu_j)
                continue;
            
        }
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        
        double svd_method = svd_V[2] / svd_V[3];
        
        it_per_id.estimated_depth = svd_method;
        
        if (it_per_id.estimated_depth < 0.1)
        {
            std::vector<int> window_id;
            it_per_id.is_outlier = true;
            it_per_id.estimated_depth = INIT_DEPTH;
            outlier_info.emplace_back(it_per_id.feature_id, window_id);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        
        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
                feature.erase(it);
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
    }
    return dep_vec;
}

int FeatureManager::getFeatureCount()
{
    int sum = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 2  && it.start_frame < WINDOW_SIZE - 2)
            //if (it.used_num >= 2)
        {
            sum++;
            //if(sum>120)
            //    return ans;
        }
    }
    return sum;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        
        if (it->start_frame != 0)
        {
            it->start_frame--;
        }
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
            {
                feature.erase(it);
                //printf("remove back\n");
            }
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;
        
        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            //it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
            {
                feature.erase(it);
                //printf("remove front\n");
            }
            
        }
        //if(it->is_margin == true)
        //    feature.erase(it);
    }
}


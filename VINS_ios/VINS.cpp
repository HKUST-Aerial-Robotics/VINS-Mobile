//
//  VINS.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "VINS.hpp"

bool LOOP_CLOSURE = true;

VINS::VINS()
:f_manager{Rs},fail_times{0},
failure_hand{false},
     drawresult{0.0, 0.0, 0.0, 0.0, 0.0, 7.0}
{
    printf("init VINS begins\n");
    t_drift.setZero();
    r_drift.setIdentity();
    clearState();
    failure_occur = 0;
    last_P.setZero();
    last_R.setIdentity();
    last_P_old.setZero();
    last_R_old.setIdentity();
}

void VINS::setIMUModel()
{
    ProjectionFactor::sqrt_info = FOCUS_LENGTH_X / 1.5 * Matrix2d::Identity();
}

void VINS::clearState()
{
    printf("clear state\n");
    for (int i = 0; i < 10 * (WINDOW_SIZE + 1); i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        IMU_linear[i].setZero();
        IMU_angular[i].setIdentity();
        pre_integrations[i] = nullptr;
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
        
        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }
    tic << TIC_X,
           TIC_Y,
           TIC_Z;
    ric = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r));
    
    frame_count = 0;
    first_imu = false;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    initProgress = 0;
    
    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;
    
    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();
    
    f_manager.clearState();
    
}

void VINS::setExtrinsic()
{
    tic << TIC_X,
           TIC_Y,
           TIC_Z;
    ric = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r));
}
void VINS::old2new()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();
        
        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();
        
        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();
        
        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic.x();
        para_Ex_Pose[i][1] = tic.y();
        para_Ex_Pose[i][2] = tic.z();
        Quaterniond q{ric};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }
    //triangulate
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
}

void VINS::new2old()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    
    Vector3d origin_P0 = Ps[0];
    
    if (failure_occur)
    {
        printf("failure recover %lf %lf %lf\n", last_P.x(), last_P.y(), last_P.z());
        origin_R0 = Utility::R2ypr(last_R_old);
        origin_P0 = last_P_old;
    }
    
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5]).toRotationMatrix());
    
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        //if ((!LOOP_CLOSURE) || (!loop_enable))
        if(true)
        {
            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);
        }
        else
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
            Vs[i] = Vector3d(para_SpeedBias[i][0],
                             para_SpeedBias[i][1],
                             para_SpeedBias[i][2]);
        }
        
        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);
        
        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }
    Vector3d cur_P0 = Ps[0];
    
    
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
    {
        dep(i) = para_Feature[i][0];
    }
    f_manager.setDepth(dep);
}

bool VINS::failureDetection()
{
    bool is_failure = false;
    
    if (f_manager.last_track_num < 4)
    {
        printf("failure little feature %d\n", f_manager.last_track_num);
        is_failure = true;
    }
    /*
    if (Bas[WINDOW_SIZE].norm() > 1)
    {
        printf("failure  big IMU acc bias estimation %f\n", Bas[WINDOW_SIZE].norm());
        is_failure = true;
    }
     */
    if (Bgs[WINDOW_SIZE].norm() > 1)
    {
        printf("failure  big IMU gyr bias estimation %f\n", Bgs[WINDOW_SIZE].norm());
        is_failure = true;
    }
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 1)
    {
        printf("failure big translation\n");
        is_failure = true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 0.5)
    {
        printf("failure  big z translation\n");
        is_failure = true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 40)
    {
        printf("failure  big delta_angle \n");
        is_failure = true;
    }
    
    if(failure_hand)
    {
        failure_hand = false;
        is_failure = true;
        printf("failure by hand!\n");
    }
    
    return is_failure;
}

/*
void VINS::failureRecover()
{
    int his_index = 0;
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(Headers_his[i] == Headers[0])
        {
            his_index = i;
            break;
        }
        if(i == WINDOW_SIZE -1)
            his_index = i;
    }
    Vector3d his_R0 = Utility::R2ypr(Rs_his[his_index]);
    
    Vector3d his_P0 = Ps_his[his_index];
    
    Vector3d cur_R0 = Utility::R2ypr(Rs[0]);
    Vector3d cur_P0 = Ps[0];
    
    double y_diff = his_R0.x() - cur_R0.x();
    
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        Rs[i] = rot_diff * Rs[i];
        Ps[i] = rot_diff * (Ps[i] - cur_P0) + his_P0;
        Vs[i] = rot_diff * Vs[i];
    }
}
 */

void VINS::reInit()
{
    failure_hand = true;
    failureDetection();
}

void VINS::update_loop_correction()
{
    //update loop correct pointcloud
    correct_point_cloud.clear();
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 4 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (/*it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 ||*/ it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d tmp = r_drift * Rs[imu_i] * (ric * pts_i + tic) + r_drift * Ps[imu_i] + t_drift;
        correct_point_cloud.push_back(tmp.cast<float>());
    }
    //update correct pose
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        Vector3d correct_p = r_drift * Ps[i] + t_drift;
        correct_Ps[i] = correct_p.cast<float>();
        Matrix3d correct_r = r_drift * Rs[i];
        correct_Rs[i] = correct_r.cast<float>();
    }
}

void VINS::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    
    if (frame_count != 0)
    {
        //covariance propagate
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        
        if(solver_flag != NON_LINEAR) //comments because of recovering
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);
        
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        
        //midpoint integration
        {
            Vector3d g{0,0,GRAVITY};
            int j = frame_count;
            Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
            Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
            Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
            Vs[j] += dt * un_acc;
        }
        
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void VINS::processImage(map<int, Vector3d> &image_msg, double header, int buf_num)
{
    int track_num;
    printf("adding feature points %lu\n", image_msg.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image_msg, track_num))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;
    
//    printf("marginalization_flag %d\n", int(marginalization_flag));
//    printf("this frame is-------------------------------%s\n", marginalization_flag ? "reject" : "accept");
//    printf("Solving %d\n", frame_count);
    printf("number of feature: %d %d\n", feature_num = f_manager.getFeatureCount(), track_num);
    
    Headers[frame_count] = header;

    if(solver_flag == INITIAL)
    {
        ImageFrame imageframe(image_msg, header);
        imageframe.pre_integration = tmp_pre_integration;
        all_image_frame.insert(make_pair(header, imageframe));
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Vector3d(0,0,0), Vector3d(0,0,0)};
        
        if (frame_count == WINDOW_SIZE)
        {
            if(track_num < 20)
            {
                clearState();
                return;
            }
            bool result = false;
            if(header - initial_timestamp > 0.3)
            {
                result = solveInitial();
                initial_timestamp = header;
            }
            if(result)
            {
                solve_ceres(buf_num);
                if(final_cost > 200)  //initialization failed, need reinitialize
                {
                    printf("final cost %lf faild!\n",final_cost);
                    delete last_marginalization_info;
                    last_marginalization_info = nullptr;
                    solver_flag = INITIAL;
                    init_status = FAIL_CHECK;
                    fail_times++;
                    slideWindow();
                }
                else
                {
                    printf("final cost %lf succ!\n",final_cost);
                    failure_occur = 0;
                    //update init progress
                    initProgress = 100;
                    init_status = SUCC;
                    fail_times = 0;
                    printf("Initialization finish---------------------------------------------------!\n");
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    f_manager.removeFailures();
                    update_loop_correction();
                    last_R = Rs[WINDOW_SIZE];
                    last_P = Ps[WINDOW_SIZE];
                    last_R_old = Rs[0];
                    last_P_old = Ps[0];
                }
            }
            else
            {
                slideWindow();
            }
        }
        else
        {
            frame_count++;
            initProgress +=2;
        }
    }
    else
    {
        bool is_nonlinear = true;
        f_manager.triangulate(Ps, tic, ric, is_nonlinear);
        solve_ceres(buf_num);
        failure_occur = 0;
        
        if (failureDetection())
        {
            failure_occur = 1;
            clearState();
            return;
        }
        slideWindow();
        f_manager.removeFailures();
        update_loop_correction();
        
        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R_old = Rs[0];
        last_P_old = Ps[0];
    }
}

void VINS::solve_ceres(int buf_num)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    
    for (int i = 0; i < NUM_OF_F; i++)
    {
        problem.AddParameterBlock(para_Feature[i], SIZE_FEATURE);
    }
    
    old2new();

    //marginalization factor
    if (last_marginalization_info != nullptr)
    {
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                     last_marginalization_parameter_blocks);
    }
    
    //IMU factor
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    
    //projection factor
    int f_m_cnt = 0;
    double f_sum = 0.0;
    double r_f_sum = 0.0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        ++feature_index;
        
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            
            f_m_cnt++;
            
            double **para = new double *[4];
            para[0] = para_Pose[imu_i];
            para[1] = para_Pose[imu_j];
            para[2] = para_Ex_Pose[0];
            para[3] = para_Feature[feature_index];
            double *res = new double[2];
            f->Evaluate(para, res, NULL);
            f_sum += sqrt(res[0] * res[0] + res[1] * res[1]);
            
            double rho[3];
            loss_function->Evaluate(res[0] * res[0] + res[1] * res[1], rho);
            r_f_sum += rho[0];
        }
    }
    visual_cost = r_f_sum;
    visual_factor_num = f_m_cnt;
    
    if(LOOP_CLOSURE)
    {
        //loop close factor
        //front_pose.measurements.clear();
        if(front_pose.header != retrive_pose_data.header)
        {
            front_pose = retrive_pose_data;  //need lock
            printf("use loop\n");
        }
        if(!front_pose.measurements.empty())
        {
            //the retrive pose is in the current window
            if(front_pose.header >= Headers[0])
            {
                //tmp_retrive_pose_buf.push(front_pose);
                printf("loop front pose  in window\n");
                for(int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(front_pose.header == Headers[i])
                    {
                        for (int k = 0; k < 7; k++)
                            front_pose.loop_pose[k] = para_Pose[i][k];
                        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                        problem.AddParameterBlock(front_pose.loop_pose, SIZE_POSE, local_parameterization);
                
                        Ps_retrive = front_pose.P_old;
                        Qs_retrive = front_pose.Q_old.toRotationMatrix();
                        int retrive_feature_index = 0;
                        int feature_index = -1;
                        int loop_factor_cnt = 0;
                        for (auto &it_per_id : f_manager.feature)
                        {
                            it_per_id.used_num = it_per_id.feature_per_frame.size();
                            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                                continue;
                            ++feature_index;
                            int start = it_per_id.start_frame;
                            //feature has been obeserved in ith frame
                            int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);
                            if(start <= i && end >=0)
                            {
                                while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
                                {
                                    retrive_feature_index++;
                                }
                                
                                if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
                                {
                                    Vector3d pts_j = Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);
                                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                    //double ratio = 1.0;
                                    //LoopClosureFactor *f = new LoopClosureFactor(pts_i, pts_j, Ps_retrive, Qs_retrive, ratio);
                                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                                    problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
                                    
                                    //printf("loop add factor %d %d %lf %lf %d\n",retrive_feature_index,feature_index,
                                    //                                         pts_j.x(), pts_i.x(),front_pose.features_ids.size());
                                    retrive_feature_index++;
                                    loop_factor_cnt++;
                                    loop_enable = true;
                                }
                                
                            }
                        }
                        printf("add %d loop factor\n", loop_factor_cnt);
                    }
                }
            }
        }
    }
    
    ceres::Solver::Options options;
    
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 1;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 10;
    //options.use_nonmonotonic_steps = true;
    if(buf_num<2)
        options.max_solver_time_in_seconds = SOLVER_TIME;
    else if(buf_num<4)
        options.max_solver_time_in_seconds = SOLVER_TIME * 2.0 / 3.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME / 2.0;
//    options.max_solver_time_in_seconds = 0.04;
    ceres::Solver::Summary summary;
    //TE(prepare_solver);
    TS(ceres);
    printf("solve\n");
    ceres::Solve(options, &problem, &summary);
    final_cost = summary.final_cost;
    //cout << summary.FullReport() << endl;
    TE(ceres);
    if(LOOP_CLOSURE)
    {
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(front_pose.header == Headers[i])
            {
                Matrix3d Rs_i = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
                Vector3d Ps_i = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
                Matrix3d Rs_loop = Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Vector3d Ps_loop = Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);
                
                front_pose.relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                front_pose.relative_q = Rs_loop.transpose() * Rs_i;
                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
            }
        }
    }
    new2old();
    
    vector<ceres::ResidualBlockId> residual_set;
    problem.GetResidualBlocks(&residual_set);
    for (auto it : residual_set)
        problem.RemoveResidualBlock(it);
    
    //for marginalization back
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        old2new();
        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        
        {
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;
                
                ++feature_index;
                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;
                
                Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                
                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    
                    Vector3d pts_j = it_per_frame.point;
                    if (imu_i == imu_j)
                        continue;
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);

                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                    vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                    vector<int>{0, 3});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                    
                }
            }
        }
        //if(LOOP_CLOSURE)
        if(false)
        {
            if(!front_pose.measurements.empty())
            {
                if(front_pose.header >= Headers[0])
                {
                    //tmp_retrive_pose_buf.push(front_pose);
                    for(int i = 0; i < WINDOW_SIZE; i++)
                    {
                        if(front_pose.header == Headers[i] && front_pose.use)
                        {
                            Ps_retrive = front_pose.P_old;
                            Qs_retrive = front_pose.Q_old.toRotationMatrix();
                            
                            int retrive_feature_index = 0;
                            int feature_index = -1;
                            int loop_factor_cnt = 0;
                            for (auto &it_per_id : f_manager.feature)
                            {
                                it_per_id.used_num = it_per_id.feature_per_frame.size();
                                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                                    continue;
                                
                                ++feature_index;
                                int start = it_per_id.start_frame;
                                //feature has been obeserved in ith frame
                                int end = (start + it_per_id.feature_per_frame.size() - i - 1);
                                if(start <= i && end >=0)
                                {
                                    while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
                                    {
                                        retrive_feature_index++;
                                    }
                                    
                                    if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
                                    {
                                        //add loop factor
                                        if(start == 0)
                                        {
                                            Vector3d pts_j = Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);
                                            Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                            
                                            LoopClosureFactor *f = new LoopClosureFactor(pts_i, pts_j, Ps_retrive, Qs_retrive, 1.0);
                                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                                           vector<double *>{para_Pose[start], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                                           vector<int>{0, 2});
                                            marginalization_info->addResidualBlockInfo(residual_block_info);
                                        }
                                        retrive_feature_index++;
                                        loop_factor_cnt++;
                                    }
                                    
                                }
                            }
                            printf("marginalize %d loop factor\n", loop_factor_cnt);
                        }
                    }
                }
            }
            
        }
        TS(per_marginalization);
        marginalization_info->preMarginalize(); //??
        TE(per_marginalization);
        TS(marginalization);
        marginalization_info->marginalize();   //??
        TE(marginalization);
        
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    //marginalize front
    else
    {
        if (last_marginalization_info&&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            old2new();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    assert(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);
                
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        
            marginalization_info->preMarginalize();
            marginalization_info->marginalize();
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
}

bool VINS::solveInitial()
{
    printf("solve initial------------------------------------------\n");
    printf("PS %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    //check imu observibility
    /*
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            if(dt == 0)
            {
                printf("init IMU variation not enouth!\n");
                init_status = FAIL_IMU;
                fail_times++;
                return false;
            }
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        cout << "aver_g " << aver_g.transpose() << endl;
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        printf("IMU variation %f!\n", var);
        if(var < 0.25)
        {
            printf("init IMU variation not enouth!\n");
            init_status = FAIL_IMU;
            fail_times++;
            return false;
        }
    }
     */
    // global sfm
    Quaterniond *Q = new Quaterniond[frame_count + 1];
    Vector3d *T = new Vector3d[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    {
        for (auto &it_per_id : f_manager.feature)
        {
            int imu_j = it_per_id.start_frame - 1;
            
            SFMFeature tmp_feature;
            tmp_feature.state = false;
            tmp_feature.id = it_per_id.feature_id;
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                Vector3d pts_j = it_per_frame.point;
                tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
            }
            sfm_f.push_back(tmp_feature);
        }
        Matrix3d relative_R;
        Vector3d relative_T;
        int l;
        if (!relativePose(0, relative_R, relative_T, l))
        {
            printf("init solve 5pts between first frame and last frame failed\n");
            return false;
        }
        //update init progress
        initProgress = 30;
        
        GlobalSFM sfm;
        if(!sfm.construct(frame_count + 1, Q, T, l,
                          relative_R, relative_T,
                          sfm_f, sfm_tracked_points))
        {
            printf("global SFM failed!");
            init_status = FAIL_SFM;
            marginalization_flag = MARGIN_OLD;
            fail_times++;
            return false;
        }
        //update init progress
        initProgress = 50;
    }
    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            cout << "key frame " << i << endl;
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * ric.transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);
        
        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            //cout << "feature id " << feature_id;
            //cout << " pts image_frame " << (i_p.second.head<2>() * 460 ).transpose() << endl;
            it = sfm_tracked_points.find(feature_id);
            if(it != sfm_tracked_points.end())
            {
                Vector3d world_pts = it->second;
                cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                pts_3_vector.push_back(pts_3);
                
                Vector2d img_pts = id_pts.second.head<2>();
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);

        if(pts_3_vector.size() < 6 )
        {
            printf("init Not enough points for solve pnp !\n");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            printf("init solve pnp fail!\n");
            init_status = FAIL_PNP;
            fail_times++;
            return false;
        }
        cv::Rodrigues(rvec, r);
        //cout << "r " << endl << r << endl;
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        //cout << "R_pnp " << endl << R_pnp << endl;
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * ric.transpose();
        frame_it->second.T = T_pnp;
    }
    delete[] Q;
    delete[] T;
    
    //update init progress
    initProgress = 75;
    
    printf("init PS after pnp %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    
    if (visualInitialAlign())
    {
        return true;
        //update init progress
        initProgress = 85;
    }
    else
    {
        init_status = FAIL_ALIGN;
        fail_times++;
        return false;
    }
    
}

bool VINS::visualInitialAlign()
{
    TS(solve_g);
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        printf("solve g failed!");
        printf("init PS alignment failed %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
        return false;
    }
    TE(solve_g);
    printf("init PS algnment succ %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }
    
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);
    
    //triangulat on cam pose , no tic
    Vector3d TIC_TMP;
    TIC_TMP.setZero();
    f_manager.triangulate(Ps, TIC_TMP, ric, true);
    
    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * tic - (s * Ps[0] - Rs[0] * tic);
    
    printf("PS after scale %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    printf("init finish--------------------\n");
    
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }
    
    Matrix3d R0 = Utility::g2R(g);
    double yaw0 = Utility::R2ypr(R0).x();
    Matrix3d yaw_refine = Utility::ypr2R(Vector3d{-yaw0,0,0});
    R0 = yaw_refine * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
        init_poses.push_back(Ps[i]);
    }
    
    return true;
}

bool VINS::relativePose(int camera_id, Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
                
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            parallax_num_view = average_parallax * 520;
            if(average_parallax * 520 < 30)
            {
                init_status = FAIL_PARALLAX;
                fail_times++;
                return false;
            }
            if(m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                printf("average_parallax %f choose l %d and newest frame to triangulate the whole structure\n", average_parallax * 520, l);
                return true;
            }
            else
            {
                init_status = FAIL_RELATIVE;
                fail_times++;
                return false;
            }
        }
    }
    return false;
}
/*
 marginalize the state from the sliding window and change feature start frame
 */
void VINS::slideWindow()
{
    //marginalize old keyframe
    if (marginalization_flag == MARGIN_OLD)
    {
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);
                std::swap(pre_integrations[i], pre_integrations[i + 1]);
                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
            
            if(pre_integrations[WINDOW_SIZE] != NULL)
            {
                delete pre_integrations[WINDOW_SIZE];
            }
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
            
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
            
            if (solver_flag == INITIAL)
            {
                double t_0 = Headers[0];
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else  //non keyframe
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
             
                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);
                
                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }
            
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];
            if(pre_integrations[WINDOW_SIZE]!=NULL)
            {
                delete pre_integrations[WINDOW_SIZE];
            }
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
            
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

void VINS::slideWindowOld()
{
    printf("marginalize back\n");
    point_cloud.clear();
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        
        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2
            &&it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d tmp = Rs[imu_i] * (ric * pts_i + tic) + Ps[imu_i];
            point_cloud.push_back(tmp.cast<float>());
        }
    }
    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric;
        R1 = Rs[0] * ric;
        P0 = back_P0 + back_R0 * tic;
        P1 = Ps[0] + Rs[0] * tic;
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();

}
void VINS::slideWindowNew()
{
    printf("marginalize front\n");
    f_manager.removeFront(frame_count);
}

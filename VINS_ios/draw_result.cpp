//
//  draw_result.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/11/16.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "draw_result.hpp"

DrawResult::DrawResult(float _pitch, float _roll, float _yaw, float _Tx, float _Ty, float _Tz)
:pitch{_pitch},roll{_roll},yaw{_yaw},Tx{_Tx},Ty{_Ty},Tz{_Tz},change_view_manualy{false}
{
    planeInit = false;
    startInit = false;
    initPoint.setZero();
    Fx = 400;
    Fy = 400;
    radius = 5.0;
    radiusAR = 20.0;
    theta = 30;
    thetaAR = 30;
    phy = -30;
    phyAR = -30;
    finger_s = finger_d = finger_p =0;
    finger_state = 0;
    origin_w.setZero();
    X0 = WIDTH/2;
    X0AR = WIDTH/2;
    Y0 = HEIGHT/2;
    Y0AR = HEIGHT/2;
    tapFlag = false;
    longPressFlag = false;
    KF_init = false;
    change_color = false;
    trajectory_color.push_back(Scalar(255, 0, 0));
    indexs.push_back(0);
    look_down = false;
    Ground_idx=0;
}
bool checkBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < HEIGHT - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < WIDTH - BORDER_SIZE;
}

float check_scale(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    float scale_factor;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    if(BORDER_SIZE <= img_x && img_x < WIDTH-10 - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < HEIGHT - 10 - BORDER_SIZE)
    {
        scale_factor = 1.0;
    }
    else
    {
        int max1 = 0,max2 = 0,max3 = 0,max4 = 0;
        
        if(img_x>WIDTH - 10)
            max1 = img_x - ( WIDTH - 10);
        
        if(img_y> HEIGHT - 10)
            max2 = img_y - (HEIGHT - 10);
        
        if(img_x<1)
            max3 = 1 - img_x;
        
        if(img_y<1)
            max4 = 1 - img_y;
        
        int max_result = max(max(max1,max2),max(max3,max4));
        
        if(max_result == max1 || max_result == max3)
            scale_factor = (float)((WIDTH-10)/2.0)/((WIDTH-10)/2.0+max_result);
        else
            scale_factor = (float)((HEIGHT-10)/2.0)/((HEIGHT-10)/2.0+max_result);
    }
    return scale_factor;
}
/*
 Convert HSV to RGB color space
 \param fR Red component, used as output, range: [0, 1]
 \param fG Green component, used as output, range: [0, 1]
 \param fB Blue component, used as output, range: [0, 1]
 \param fH Hue component, used as input, range: [0, 360]
 \param fS Hue component, used as input, range: [0, 1]
 \param fV Hue component, used as input, range: [0, 1]
 */

cv::Scalar DrawResult::newColor() {
    
    float ratio = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);;
    ratio += 0.618033988749895; //golden_ratio_conjugate
    ratio = fmod(ratio , 1.0);
    
    float fR, fG, fB, fH, fS, fV;
    fH = ratio*360;
    fS = 0.7;
    fV = 0.95;
    float fC = fV * fS; // Chroma
    float fHPrime = fmod(fH / 60.0, 6);
    float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
    float fM = fV - fC;
    
    if(0 <= fHPrime && fHPrime < 1) {
        fR = fC;
        fG = fX;
        fB = 0;
    } else if(1 <= fHPrime && fHPrime < 2) {
        fR = fX;
        fG = fC;
        fB = 0;
    } else if(2 <= fHPrime && fHPrime < 3) {
        fR = 0;
        fG = fC;
        fB = fX;
    } else if(3 <= fHPrime && fHPrime < 4) {
        fR = 0;
        fG = fX;
        fB = fC;
    } else if(4 <= fHPrime && fHPrime < 5) {
        fR = fX;
        fG = 0;
        fB = fC;
    } else if(5 <= fHPrime && fHPrime < 6) {
        fR = fC;
        fG = 0;
        fB = fX;
    } else {
        fR = 0;
        fG = 0;
        fB = 0;
    }
    
    fR += fM;
    fG += fM;
    fB += fM;
    
    return Scalar( fR*255, fG*255, fB*255 );
}
vector<Vector3f> DrawResult::calculate_camera_pose(Vector3f camera_center, Matrix3f Rc, float length)
{
    vector<Vector3f> result;
    vector<Vector3f> origin;
    origin.push_back(Vector3f(length/2.0,length/2.0,-length*0.7));
    origin.push_back(Vector3f(-length/2.0,length/2.0,-length*0.7));
    origin.push_back(Vector3f(-length/2.0,-length/2.0,-length*0.7));
    origin.push_back(Vector3f(length/2.0,-length/2.0,-length*0.7));
    origin.push_back(Vector3f(0,0,0));
    result.clear();
    
    Eigen::Matrix3f RIC;
    RIC = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r)).cast<float>();
    
    for (auto it : origin)
    {
        Vector3f tmp;
        tmp = Rc*it + camera_center;
        /*
         Eigen::Vector3f Pc;
         cv::Point2f pts;
         Pc = R_c_w * RIC.transpose() * tmp + T_c_w;
         pts.x = Fx * Pc.x() / Pc.z()+ PX;
         pts.y = Fy * Pc.y() / Pc.z()+ PY;
         */
        result.push_back(tmp);
    }
    return result;
}

Vector4f creatPlane(Vector3f p1,Vector3f p2, Vector3f p3)
{
    Vector3f arrow1 = p2 - p1;
    Vector3f arrow2 = p3 - p1;
    Vector3f normal;
    normal = arrow1.cross(arrow2);
    return Vector4f(normal.x() , normal.y(), normal.z(),
                    -normal.x() * p1.x() - normal.y() * p1.y() - normal.z() * p1.z());
}

//Ax+By+Cz+D = 0;
Vector4f DrawResult::findPlane(vector<Vector3f> &point_cloud)
{
    int K = 2000, k = 1;    //max iterate num and current iterate num
    float sigma = 0.01;
    int pretotal = 0;
    Vector4f bestplane;
    bestplane << 0,0,0,0;
    
    while(pretotal<point_cloud.size()/2 && k < K)
    {
        int index1 = rand() % point_cloud.size();
        int index2 = rand() % point_cloud.size();
        int index3 = rand() % point_cloud.size();
        while(index2 == index1)
            index2 = rand() % point_cloud.size();
        while(index3 == index2 || index3 == index1)
            index3 = rand() % point_cloud.size();
        Vector3f point1,point2,point3;
        point1 = point_cloud[index1];
        point2 = point_cloud[index2];
        point3 = point_cloud[index3];
        
        Vector4f plane = creatPlane(point1, point2, point3);
        int inPlaneNum = 0;
        Vector3f point_sum;
        point_sum.setZero();
        for (int i = 0; i< point_cloud.size(); i++)
        {
            if(i == index1 || i == index2 || i == index3)
                continue;
            Vector3f tmp;
            tmp << plane(0),plane(1),plane(2);
            float dist = fabs(tmp.dot(point_cloud[i]) + plane(3))/(tmp).norm();
            if(dist < sigma)
            {
                inPlaneNum++;
                point_sum += point_cloud[i];
            }
        }
        if(inPlaneNum > pretotal)
        {
            pretotal = inPlaneNum;
            bestplane = plane;
            initPoint = point_sum/inPlaneNum;
        }
        k++;
    }
    std::cout << "Plane: " << pretotal << " iter:" << k<< std::endl;
    return bestplane;
}

Vector3f DrawResult::findGround(vector<Vector3f> &point_cloud, vector<Vector3f> &inlier_points)
{
    if(!look_down)
        return Vector3f(0,0,0);
    int height_range[30];
    double height_sum[30];
    vector<vector<Vector3f>> points_clusters;
    points_clusters.resize(30);
    for (int i = 0; i < 30; i++)
    {
        height_range[i] = 0;
        height_sum[i] = 0;
    }
    for (unsigned int i = 0; i < point_cloud.size(); i++)
    {
        double z = point_cloud[i].z();
        int index = (z + 2.0) / 0.1;
        if (0 <= index && index < 30)
        {
            height_range[index]++;
            height_sum[index] += z;
            points_clusters[index].push_back(point_cloud[i]);
        }
    }
    int max_num = 0;
    int max_index = -1;
    for (int i = 1; i < 29; i++)
    {
        if (max_num < height_range[i])
        {
            max_num = height_range[i];
            max_index = i;
        }
    }
    if (max_index == -1)
        return Vector3f(0,0,0);
    else
    {
        inlier_points = points_clusters[max_index];
        Vector3f tmp_p;
        tmp_p.setZero();
        for(int i = 0; i< inlier_points.size(); i++)
        {
            tmp_p += inlier_points[i];
        }
        return tmp_p/inlier_points.size();
    }
}

Vector3f DrawResult::findZfromXY(Vector3f point, Vector4f plane)
{
    Vector3f pointInPlane;
    Vector3f n_plane;
    n_plane << plane(0),plane(1),plane(2);
    float tmp = (n_plane.norm());
    float t = (n_plane.dot(point) + plane(3))/(tmp*tmp);
    pointInPlane << point(0) - plane(0)*t,
    point(1) - plane(1)*t,
    point(2) - plane(2)*t;
    return pointInPlane;
}

/*
 * output: model position in vins frame
 */
void DrawResult::computeAR(vector<Vector3f> &point_cloud, Vector3f &model)
{
    if(point_cloud.size() < 10)
        return;
    /*
     startInit = true;
     if(!planeInit && startInit)
     {
     initPlane = findPlane(point_cloud);
     
     model = findZfromXY(initPoint,initPlane);
     planeInit = true;
     }
     */
    vector<Vector3f> inlier_points;
    int height_range[30];
    double height_sum[30];
    vector<vector<Vector3f>> points_clusters;
    points_clusters.resize(30);
    for (int i = 0; i < 30; i++)
    {
        height_range[i] = 0;
        height_sum[i] = 0;
    }
    for (unsigned int i = 0; i < point_cloud.size(); i++)
    {
        double z = point_cloud[i].z();
        int index = (z + 2.0) / 0.1;
        if (0 <= index && index < 30)
        {
            height_range[index]++;
            height_sum[index] += z;
            points_clusters[index].push_back(point_cloud[i]);
        }
    }
    int max_num = 0;
    int max_index = -1;
    for (int i = 1; i < 29; i++)
    {
        if (max_num < height_range[i])
        {
            max_num = height_range[i];
            max_index = i;
        }
    }
    if (max_index == -1)
        return;
    else
    {
        inlier_points = points_clusters[max_index];
        Vector3f tmp_p;
        tmp_p.setZero();
        for(int i = 0; i< inlier_points.size(); i++)
        {
            tmp_p += inlier_points[i];
        }
        model = tmp_p/inlier_points.size();
        planeInit = true;
        return;
    }
    
}

/*
 draw a real box in (0,0,1.5) of world frame
 */

void DrawResult::drawGround(cv::Mat &result, vector<Vector3f> &point_cloud, Vector3f P_latest, Matrix3f R_latest)
{
    Eigen::Matrix3f RIC;
    RIC = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r)).cast<float>();
    cv::Mat aa(HEIGHT,WIDTH,CV_8UC3,Scalar(0,0,0));
    result = aa;
    
    std::vector<Vec2f_> points;
    for (unsigned int i = 0; i < point_cloud.size(); i++)
    {
        Vector3f Pc;
        Pc = (R_latest * RIC).transpose()* (point_cloud[i] - 1.0*P_latest  - R_latest * Vector3f(TIC_X,TIC_Y,TIC_Z));
        
        cv::Point2f pts;
        pts.x = FOCUS_LENGTH_X * Pc.x() / Pc.z()+ PX;
        pts.y = FOCUS_LENGTH_Y * Pc.y() / Pc.z()+ PY;
        
        points.push_back(Vec2f_(pts.x, pts.y));
        
    }
    Delaunay triangulation;
    std::vector<Triangle> triangles = triangulation.triangulate(points);
    //std::cout << triangles.size() << " triangles generated\n";
    std::vector<Edge> edges = triangulation.getEdges();
    
    for(auto e = begin(edges); e != end(edges); e++) {
        cv::Point2f pts, pts2;
        pts.x=(*e).p1.x;
        pts.y=(*e).p1.y;
        pts2.x=(*e).p2.x;
        pts2.y=(*e).p2.y;
        cv::line(result, pts, pts2, cvScalar(100,100,100), 1, 8, 0);
    }
    
}

void DrawResult::drawBox(cv::Mat &result, Vector3f corner_0, Vector3f corner_x, Vector3f corner_y, Vector3f corner_z, float size, Vector3f P_latest, Matrix3f R_latest, bool inAR)
{
    
    Eigen::Matrix3f RIC;
    RIC = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r)).cast<float>();
    
    vector<Vector3f> boxConers;
    boxConers.push_back(corner_0);
    boxConers.push_back(corner_x);
    boxConers.push_back(corner_y);
    boxConers.push_back(corner_x + corner_y - corner_0);
    
    boxConers.push_back(corner_z);
    boxConers.push_back(corner_x + corner_z - corner_0);
    boxConers.push_back(corner_y + corner_z - corner_0);
    boxConers.push_back(corner_x + corner_y + corner_z - 2.0*corner_0);
    
    vector<cv::Point2f> boxImage;
    Eigen::Vector3f Pc;
    vector<float> depth_of_coner;
    for(auto it : boxConers)
    {
        if (inAR)
            Pc = (R_latest * RIC).transpose()* (it - 1.0*P_latest  - R_latest * Vector3f(TIC_X,TIC_Y,TIC_Z));
        else
            Pc = R_latest.transpose() * (it - origin_w - P_latest);
        if(Pc.z()<0)
            return;
        
        cv::Point2f pts;
        if (inAR)
        {
            pts.x = FOCUS_LENGTH_X * Pc.x() / Pc.z()+ PX;
            pts.y = FOCUS_LENGTH_Y * Pc.y() / Pc.z()+ PY;
        }
        else{
            pts.x = Fx * Pc.x() / Pc.z()+ Y0;
            pts.y = Fy * Pc.y() / Pc.z()+ X0;
        }
        depth_of_coner.push_back(Pc.norm());
        boxImage.push_back(pts);
    }
    if(Pc.z() <0)
        return;
    
    
    //draw color
    cv::Point* p = new cv::Point[8];
    p[0] = boxImage[0];
    p[1] = boxImage[1];
    p[2] = boxImage[2];
    p[3] = boxImage[3];
    p[4] = boxImage[4];
    p[5] = boxImage[5];
    p[6] = boxImage[6];
    p[7] = boxImage[7];
    
    int npts[1] = {4};
    float min_depth = 10;
    int min_index = 5;
    for(int i= 0; i< depth_of_coner.size(); i++)
    {
        if(depth_of_coner[i] < min_depth)
        {
            min_depth = depth_of_coner[i];
            min_index = i;
        }
    }
    
    cv::Point plain[1][4];
    const cv::Point* ppt[1] = {plain[0]};
    //first draw large depth plane
    int point_group[8][12] = {{0,1,5,4, 0,4,6,2, 0,1,3,2},
        {0,1,5,4, 1,5,7,3, 0,1,3,2},
        {2,3,7,6, 0,4,6,2, 0,1,3,2},
        {2,3,7,6, 1,5,7,3, 0,1,3,2},
        {0,1,5,4, 0,4,6,2, 4,5,7,6},
        {0,1,5,4, 1,5,7,3, 4,5,7,6},
        {2,3,7,6, 0,4,6,2, 4,5,7,6},
        {2,3,7,6, 1,5,7,3, 4,5,7,6}};
    float aver_depth = 0;
    
    plain[0][0] = p[point_group[min_index][4]];
    plain[0][1] = p[point_group[min_index][5]];
    plain[0][2] = p[point_group[min_index][6]];
    plain[0][3] = p[point_group[min_index][7]];
    cv::fillPoly(result, ppt, npts, 1, cv::Scalar(0, 200, 0));
    
    plain[0][0] = p[point_group[min_index][0]];
    plain[0][1] = p[point_group[min_index][1]];
    plain[0][2] = p[point_group[min_index][2]];
    plain[0][3] = p[point_group[min_index][3]];
    cv::fillPoly(result, ppt, npts, 1, cv::Scalar(200, 0, 0));
    
    if(depth_of_coner[point_group[min_index][2]] + depth_of_coner[point_group[min_index][3]] >
       depth_of_coner[point_group[min_index][5]] + depth_of_coner[point_group[min_index][6]])
    {
        plain[0][0] = p[point_group[min_index][4]];
        plain[0][1] = p[point_group[min_index][5]];
        plain[0][2] = p[point_group[min_index][6]];
        plain[0][3] = p[point_group[min_index][7]];
        cv::fillPoly(result, ppt, npts, 1, cv::Scalar(0, 200, 0));
        
    }
    plain[0][0] = p[point_group[min_index][8]];
    plain[0][1] = p[point_group[min_index][9]];
    plain[0][2] = p[point_group[min_index][10]];
    plain[0][3] = p[point_group[min_index][11]];
    cv::fillPoly(result, ppt, npts, 1, cv::Scalar(0, 0, 200));
}

void DrawResult::drawAR(cv::Mat &result, vector<Vector3f> &point_cloud, Vector3f P_latest, Matrix3f R_latest)
{
    cv::Mat aa(HEIGHT,WIDTH,CV_8UC3,Scalar(0,0,0));
    result = aa;
    Eigen::Matrix3f RIC;
    RIC = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r)).cast<float>();
    
    Vector3f cam_z(0, 0, -1);
    Vector3f w_cam_z = R_latest * RIC * cam_z;
    if (acos(w_cam_z.dot(Vector3f(0, 0, 1))) * 180.0 / M_PI < 60)
    {
        printf("look down\n");
        look_down = true;
    }
    else
    {
        printf("not down\n");
        look_down = false;
    }
    
    Vector3f groundPlanePoint;
    vector<Vector3f> point_inlier;
    groundPlanePoint = findGround(point_cloud, point_inlier);
    //printf("Ground inlier size %d\n", int(point_inlier.size()) );
    
    
    ///draw ground area
    if (point_inlier.size()>28)
        drawGround(result, point_inlier, P_latest, R_latest);
    
    //draw existing boxes
    for (unsigned int i =0; i< Grounds.size(); i++)
    {
        if ( Grounds[i].boxflag)
            drawBox(result, Grounds[i].ori, Grounds[i].cox, Grounds[i].coy, Grounds[i].coz, Grounds[i].size, P_latest, R_latest, true);
    }
    
    
    
    
    ////////////////////////////// translation response
    /////////////////////////// follow mode translation response
    if ( locationX != locationX_p or locationY != locationY_p )
    {
        float xx=480 - locationY -1;
        float yy= locationX;
        float cx=240;
        float cy=320;
        Vector3f box_center, Pc;
        Vector2f box_center_xy, center_input;
        center_input<< xx, yy;
        
        float distance, dis_min;
        unsigned int dis_min_idx;
        dis_min= 100000;
        for (unsigned int i =0; i< Grounds.size(); i++)
        {
            Pc = (R_latest * RIC).transpose()* (Grounds[i].center - 1.0*P_latest  - R_latest * Vector3f(TIC_X,TIC_Y,TIC_Z));
            box_center_xy.x() = FOCUS_LENGTH_X * Pc.x() / Pc.z()+ PX;
            box_center_xy.y() = FOCUS_LENGTH_Y * Pc.y() / Pc.z()+ PY;
            distance = (box_center_xy - center_input).norm();
            if (distance < dis_min)
            {
                dis_min = distance;
                dis_min_idx = i;
            }
        }
        
        
        if ( dis_min < 120 and Grounds.size()>0 and Grounds[dis_min_idx].moveflag and finger_state == 1)
        {
            Vector3f rotation_axis;
            rotation_axis<<cy-yy, xx-cx, 0;
            rotation_axis = rotation_axis / rotation_axis.norm();
            Quaternionf rotation_q;
            float rotation_angle;
            rotation_angle = atan2(sqrt( (cy-yy)*(cy-yy) + (xx-cx)*(xx-cx) ), FOCUS_LENGTH_X);
            rotation_q.x()=rotation_axis.x() * sin(rotation_angle /2.0);
            rotation_q.y()=rotation_axis.y() * sin(rotation_angle /2.0);
            rotation_q.z()=rotation_axis.z() * sin(rotation_angle /2.0);
            rotation_q.w()=cos(rotation_angle /2.0);
            Matrix3f R_CP, RWC, RWP;
            R_CP=rotation_q;
            RWC = R_latest * RIC;
            RWP = RWC * R_CP;
            
            if (RWP(2,2) < 0 )
            {
                Vector3f unit_direction, direction;
                unit_direction = RWP*Vector3f(0,0,1);
                float length_direction;
                Vector3f plane_normal(Grounds[dis_min_idx].initPlane[0], Grounds[dis_min_idx].initPlane[1], Grounds[dis_min_idx].initPlane[2]);
                length_direction = abs(-1.0* (plane_normal.dot(P_latest) + Grounds[dis_min_idx].initPlane[3]) / (plane_normal.dot(unit_direction)) );
                direction = unit_direction * length_direction;
                
                if (length_direction<20)
                {
                    Grounds[dis_min_idx].ori = direction + P_latest - (Grounds[dis_min_idx].lix*Grounds[dis_min_idx].size + Grounds[dis_min_idx].liy * Grounds[dis_min_idx].size) /2.0;
                    Grounds[dis_min_idx].cox = Grounds[dis_min_idx].ori + Grounds[dis_min_idx].lix*Grounds[dis_min_idx].size;
                    Grounds[dis_min_idx].coy = Grounds[dis_min_idx].ori + Grounds[dis_min_idx].liy*Grounds[dis_min_idx].size;
                    Grounds[dis_min_idx].coz = Grounds[dis_min_idx].ori + Grounds[dis_min_idx].liz*Grounds[dis_min_idx].size;
                    Grounds[dis_min_idx].center = (Grounds[dis_min_idx].cox + Grounds[dis_min_idx].coy)/2;
                }
            }
        }
        
        locationX_p = locationX;
        locationY_p = locationY;
    }
    
    ////////////////////////////// translation response
    
    ////////////////////////////// scale response
    {
        float xx=480 - locationYP -1;
        float yy= locationXP;
        
        Vector3f box_center, Pc;
        Vector2f box_center_xy, center_input;
        center_input<< xx, yy;
        
        
        float distance, dis_min;
        unsigned int dis_min_idx;
        dis_min= 100000;
        for (unsigned int i =0; i< Grounds.size(); i++)
        {
            Pc = (R_latest * RIC).transpose()* (Grounds[i].center - 1.0*P_latest  - R_latest * Vector3f(TIC_X,TIC_Y,TIC_Z));
            box_center_xy.x() = FOCUS_LENGTH_X * Pc.x() / Pc.z()+ PX;
            box_center_xy.y() = FOCUS_LENGTH_Y * Pc.y() / Pc.z()+ PY;
            distance = (box_center_xy - center_input).norm();
            if (distance < dis_min)
            {
                dis_min = distance;
                dis_min_idx = i;
            }
        }
        
        if (radiusAR!= radius_p)
        {
            if ( dis_min< 120 and finger_state ==3 and Grounds.size()>0)
            {
                Grounds[dis_min_idx].size = Grounds[dis_min_idx].size * (1 + (radius_p - radiusAR)*0.008);
                
                Grounds[dis_min_idx].ori = (Grounds[dis_min_idx].cox + Grounds[dis_min_idx].coy)/2 - (Grounds[dis_min_idx].lix*Grounds[dis_min_idx].size + Grounds[dis_min_idx].liy * Grounds[dis_min_idx].size) /2.0;
                Grounds[dis_min_idx].cox = Grounds[dis_min_idx].ori + Grounds[dis_min_idx].lix*Grounds[dis_min_idx].size;
                Grounds[dis_min_idx].coy = Grounds[dis_min_idx].ori + Grounds[dis_min_idx].liy*Grounds[dis_min_idx].size;
                Grounds[dis_min_idx].coz = Grounds[dis_min_idx].ori + Grounds[dis_min_idx].liz*Grounds[dis_min_idx].size;
            }
            radius_p= radiusAR;
        }
        
    }
    ////////////////////////////// scale response
    
    
    ///////////////////////////// rotation response
    {
        
        float xx=480 - locationYT2 -1;
        float yy= locationXT2;
        
        Vector3f box_center, Pc;
        Vector2f box_center_xy, center_input;
        center_input<< xx, yy;
        
        float distance, dis_min;
        unsigned int dis_min_idx;
        dis_min= 100000;
        for (unsigned int i =0; i< Grounds.size(); i++)
        {
            Pc = (R_latest * RIC).transpose()* (Grounds[i].center - 1.0*P_latest  - R_latest * Vector3f(TIC_X,TIC_Y,TIC_Z));
            box_center_xy.x() = FOCUS_LENGTH_X * Pc.x() / Pc.z()+ PX;
            box_center_xy.y() = FOCUS_LENGTH_Y * Pc.y() / Pc.z()+ PY;
            distance = (box_center_xy - center_input).norm();
            if (distance < dis_min)
            {
                dis_min = distance;
                dis_min_idx = i;
            }
        }
        
        
        
        if (X0AR!=X0_p or Y0AR != Y0_p)
        {
            if ( dis_min < 120 and Grounds.size()>0 )
            {
                Matrix3f RWC = R_latest * RIC;
                
                Vector3f h_r;
                h_r<<X0_p - X0AR, Y0AR - Y0_p, 0;
                h_r = RWC * h_r;
                Vector2f h_hori, h_r_hori;
                h_hori << RWC(1,2), -RWC(0,2);
                h_r_hori << h_r.x(), h_r.y();
                
                ///////////
                Matrix3f Ra;
                float d;
                d=(h_hori.x()*h_r_hori.x() + h_hori.y()*h_r_hori.y()) * 0.02;
                Ra<< cos (d), -sin(d), 0,
                sin(d), cos(d), 0,
                0,0,1;
                Vector3f TT;
                TT=Grounds[dis_min_idx].center;
                Vector3f aa, bb, cc, dd;
                aa= Grounds[dis_min_idx].ori - TT;
                bb= Grounds[dis_min_idx].cox - TT;
                cc= Grounds[dis_min_idx].coy - TT;
                dd= Grounds[dis_min_idx].coz - TT;
                Grounds[dis_min_idx].ori = Ra*aa+TT;
                Grounds[dis_min_idx].cox = Ra*bb+TT;
                Grounds[dis_min_idx].coy = Ra*cc+TT;
                Grounds[dis_min_idx].coz = Ra*dd+TT;
                Grounds[dis_min_idx].lix = (Grounds[dis_min_idx].cox - Grounds[dis_min_idx].ori)/Grounds[dis_min_idx].size;
                Grounds[dis_min_idx].liy = (Grounds[dis_min_idx].coy - Grounds[dis_min_idx].ori)/Grounds[dis_min_idx].size;
                Grounds[dis_min_idx].liz = (Grounds[dis_min_idx].coz - Grounds[dis_min_idx].ori)/Grounds[dis_min_idx].size;
                
            }
            X0_p=X0AR;
            Y0_p= Y0AR;
        }
        
        
    }
    ///////////////////////////// rotation response
    
    //////////////////////////// long press unlock response
    if (longPressFlag)
    {
        float xx=480 - locationLongPressY -1;
        float yy= locationLongPressX;
        
        Vector3f box_center, Pc;
        Vector2f box_center_xy, center_input;
        center_input<< xx, yy;
        
        
        float distance, dis_min;
        unsigned int dis_min_idx;
        dis_min= 100000;
        for (unsigned int i =0; i< Grounds.size(); i++)
        {
            Pc = (R_latest * RIC).transpose()* (Grounds[i].center - 1.0*P_latest  - R_latest * Vector3f(TIC_X,TIC_Y,TIC_Z));
            box_center_xy.x() = FOCUS_LENGTH_X * Pc.x() / Pc.z()+ PX;
            box_center_xy.y() = FOCUS_LENGTH_Y * Pc.y() / Pc.z()+ PY;
            distance = (box_center_xy - center_input).norm();
            if (distance < dis_min)
            {
                dis_min = distance;
                dis_min_idx = i;
            }
        }
        
        if ( dis_min < 120 and Grounds.size()>0 )
        {
            Grounds[dis_min_idx].moveflag = true;
            ////show something
            cv::Point2f pts1;
            
            Pc = (R_latest * RIC).transpose()* (Grounds[dis_min_idx].coz - 1.0*P_latest  - R_latest * Vector3f(TIC_X,TIC_Y,TIC_Z));
            pts1.x = FOCUS_LENGTH_X * Pc.x() / Pc.z()+ PX;
            pts1.y = FOCUS_LENGTH_Y * Pc.y() / Pc.z()+ PY;
            
            cv::circle(result, pts1, 0, cvScalar(0,255,0), 12);
        }
        longPressFlag = false;
    }
    //////////////////////////// long press unlock response
    
    //add new box
    if (tapFlag and point_inlier.size()>28)
    {
        float xx = 480 - locationTapY -1;
        float yy = locationTapX;
        Vector3f Pc;
        Vector2f box_center_xy, center_input;
        center_input<< xx, yy;
        Pc = (R_latest * RIC).transpose()* (groundPlanePoint - 1.0*P_latest  - R_latest * Vector3f(TIC_X,TIC_Y,TIC_Z));
        box_center_xy.x() = FOCUS_LENGTH_X * Pc.x() / Pc.z()+ PX;
        box_center_xy.y() = FOCUS_LENGTH_Y * Pc.y() / Pc.z()+ PY;
        
        float distance;
        bool security_flag=true;
        Vector2f box_center_his;
        for (unsigned int i =0; i< Grounds.size(); i++)
        {
            Pc = (R_latest * RIC).transpose()* (Grounds[i].center - 1.0*P_latest  - R_latest * Vector3f(TIC_X,TIC_Y,TIC_Z));
            box_center_his.x() = FOCUS_LENGTH_X * Pc.x() / Pc.z()+ PX;
            box_center_his.y() = FOCUS_LENGTH_Y * Pc.y() / Pc.z()+ PY;
            distance = (box_center_his - box_center_xy).norm();
            if (distance < 150)
            {
                security_flag = false;
            }
        }
        
        
        if ( (box_center_xy - center_input).norm()<150 and security_flag)
        {
            Vector3f ori, cox, coy, coz;
            Vector3f linex, liney, linez;
            float lengthc;
            
            lengthc = (P_latest - groundPlanePoint).norm()*0.25;
            linex = Vector3f(1,0,0);
            liney = Vector3f(0,1,0);
            linez = Vector3f(0,0,1);
            ori = groundPlanePoint - (linex*lengthc + liney*lengthc) /2.0;
            
            cox = ori + linex*lengthc;
            coy = ori + liney*lengthc;
            coz = ori + linez*lengthc;
            
            drawBox(result, ori, cox, coy, coz, lengthc, P_latest, R_latest, true);
            GroundPoint gp(Ground_idx ++, groundPlanePoint);
            
            gp.boxflag = true;
            gp.ori = ori;
            gp.cox = cox;
            gp.coy = coy;
            gp.coz = coz;
            gp.lix = linex;
            gp.liy = liney;
            gp.liz = linez;
            gp.size = lengthc;
            gp.initPlane = findPlane(point_inlier);
            Grounds.push_back(gp);
            
        }
        tapFlag = false;
    }
    
    
}

/////draw existing boxes in virtual camera
void DrawResult::drawBoxVirturCam(cv::Mat &result)
{
    Vector3f camInWorld_T;
    if(phy > 89)
        phy = 89;
    else if(phy < -89)
        phy = -89;
    if(theta > 89)
        theta = 89;
    else if(theta < -89)
        theta = -89;
    
    camInWorld_T.z() = radius * sin(theta * C_PI/ 180.0);
    camInWorld_T.x() = -radius * cos(theta * C_PI/ 180.0) * sin(phy* C_PI/ 180.0);
    camInWorld_T.y() = -radius * cos(theta* C_PI/ 180.0) * cos(phy* C_PI/ 180.0);
    Matrix3f camInWorld_R;
    Vector3f Zwc = -camInWorld_T/camInWorld_T.lpNorm<2>();
    Vector3f Xwc;
    Xwc  << 1.0, -camInWorld_T.x()/camInWorld_T.y(), 0;
    Xwc = Xwc/Xwc.lpNorm<2>();
    Vector3f Ywc = Zwc.cross(Xwc);
    Ywc = Ywc/Ywc.lpNorm<2>();
    
    camInWorld_R << Xwc.x(),Ywc.x(),Zwc.x(),
    Xwc.y(),Ywc.y(),Zwc.y(),
    Xwc.z(),Ywc.z(),Zwc.z();
    
    
    for (unsigned int i =0; i< Grounds.size(); i++)
    {
        if ( Grounds[i].boxflag)
        {
            drawBox(result, Grounds[i].ori, Grounds[i].cox, Grounds[i].coy, Grounds[i].coz, Grounds[i].size, camInWorld_T, camInWorld_R, false);
        }
    }
}

/*
 Reproject Pw to image plane of a virtual camera.
 Tx,Ty,Tz is virtual camera translation in real camera frame, which is constant.
 pitch,roll,yaw is virtual camera rotation in real camera frame, which is changed by user touch screen
 
 Pw is all of the point in world frame, which include real camera (iphone: start from 0) pose and point clouds.
 
 Pc = RIC^t * (Pw - TIC); rotate from world frame to real camera frame
 Pv = RVC^t * Pc + TVC;
 
 
 */
cv::Point2f DrawResult::World2VirturCam(Eigen::Vector3f xyz, float &depth)
{
    Vector3f camInWorld_T;
    if(phy > 89)
        phy = 89;
    else if(phy < -89)
        phy = -89;
    if(theta > 89)
        theta = 89;
    else if(theta < -89)
        theta = -89;
    
    camInWorld_T.z() = radius * sin(theta * C_PI/ 180.0);
    camInWorld_T.x() = -radius * cos(theta * C_PI/ 180.0) * sin(phy* C_PI/ 180.0);
    camInWorld_T.y() = -radius * cos(theta* C_PI/ 180.0) * cos(phy* C_PI/ 180.0);
    Matrix3f camInWorld_R;
    /*make sure camera optical axis is towards to world origin
     camInWorld_T = -camInWorld_R * (0, 0, 1)^T
     */
    //camInWorld_R = Utility::ypr2R(Vector3f(0, 0, -theta)) * Utility::ypr2R(Vector3f(0, phy, 0)) * Utility::ypr2R(Vector3f(0, 0, -90));
    Vector3f Zwc = -camInWorld_T/camInWorld_T.lpNorm<2>();
    Vector3f Xwc;
    Xwc  << 1.0, -camInWorld_T.x()/camInWorld_T.y(), 0;
    Xwc = Xwc/Xwc.lpNorm<2>();
    Vector3f Ywc = Zwc.cross(Xwc);
    Ywc = Ywc/Ywc.lpNorm<2>();
    
    camInWorld_R << Xwc.x(),Ywc.x(),Zwc.x(),
    Xwc.y(),Ywc.y(),Zwc.y(),
    Xwc.z(),Ywc.z(),Zwc.z();
    
    Vector3f Pc = camInWorld_R.transpose() * (xyz - origin_w - camInWorld_T);
    
    cv::Point2f pts;
    pts.x = Fx * Pc.x() / Pc.z()+ Y0;
    pts.y = Fy * Pc.y() / Pc.z()+ X0;
    depth = Pc.z();
    return pts;
}

void DrawResult::Reprojection(cv::Mat &result, vector<Vector3f> &point_cloud, const Matrix3f *R_window,const Vector3f *T_window, bool box_in_trajectorty)
{
    float depth_marker;
    cv::Mat aa(WIDTH,HEIGHT,CV_8UC3,Scalar(242,242,242));
    result = aa;
    
    Eigen::Matrix3f RIC;
    RIC = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r)).cast<float>();
    //std::cout << RIC << std::endl;
    Eigen::Matrix3f R_v_c = Utility::ypr2R(Eigen::Vector3f{yaw, pitch, roll});
    Eigen::Vector3f T_v_c;
    T_v_c << Tx,
    Ty,
    Tz;
    
    cv::Point2f pts_pre;
    cv::Point2f pts;
    
    for (int i=0; i<pose.size(); i++)
    {
        Eigen::Vector3f Pc;
        pts = World2VirturCam(pose[i], depth_marker);
        if(i == 0)
        {
            pts_pre = pts;
            continue;
        }
        
        while(trajectory_color.size() <= segment_indexs[i])
            trajectory_color.push_back(newColor());
        cv::line(result, pts_pre, pts, trajectory_color[segment_indexs[i]], 2, 8, 0);
        
        pts_pre = pts;
    }
    //draw frame arrow
    {
        Vector3f p1, p2;
        cv::Point2f pt1, pt2;
        float length = 2.4 * 400 * radius / (Fx * 5.0) ;
        float scale_factor;
        p1 << -length, 0, 0;
        p2 << length, 0, 0;
        pt1 = World2VirturCam(p1, depth_marker);
        pt2 = World2VirturCam(p2, depth_marker);
        
        arrowedLine(result, pt1, pt2, cvScalar(100,100,100),1, 8, 0, 0.02);
        cv::putText(result, "X", pt2, 0, 0.5, cvScalar(100,100,100));
        
        p1 << 0, -length, 0;
        p2 << 0, length, 0;
        pt1 = World2VirturCam(p1, depth_marker);
        pt2 = World2VirturCam(p2, depth_marker);
        
        arrowedLine(result, pt1, pt2, cvScalar(100,100,100),1 , 8, 0, 0.02);
        cv::putText(result, "Y", pt2, 0, 0.5, cvScalar(100,100,100));
        
        p1 << 0, 0, -length;
        p2 << 0, 0, length;
        pt1 = World2VirturCam(p1, depth_marker);
        pt2 = World2VirturCam(p2, depth_marker);
        
        arrowedLine(result, pt1, pt2, cvScalar(100,100,100), 1, 8, 0, 0.02);
        cv::putText(result, "Z", pt2, 0, 0.5, cvScalar(100,100,100));
        
        //draw grid
        {
            float dis = 1.0;
            int line_num = 9;
            vector<pair<Vector3f, Vector3f>> grid_space;
            vector<pair<Point2f, Point2f>> grid_plane;
            Vector3f origin_grid;
            origin_grid << - dis*(line_num/2),
            - dis*(line_num/2),
            0;
            for(int i=0; i < line_num; i++)
            {
                pair<Vector3f, Vector3f> tmp_Pts;
                tmp_Pts.first = origin_grid + Vector3f(dis * i, 0, 0);
                tmp_Pts.second = origin_grid + Vector3f(dis * i, dis*(line_num - 1), 0);
                grid_space.push_back(tmp_Pts);
                
                tmp_Pts.first = origin_grid + Vector3f(0, dis * i, 0);
                tmp_Pts.second = origin_grid + Vector3f(dis*(line_num - 1), dis * i, 0);
                grid_space.push_back(tmp_Pts);
            }
            for(auto it : grid_space)
            {
                cv::Point2f pts;
                pts = World2VirturCam(it.first, depth_marker);
                
                cv::Point2f pts2;
                pts2 = World2VirturCam(it.second, depth_marker);
                cv::line(result, pts, pts2, cvScalar(180,180,180), 1, 8, 0);
            }
        }
    }
    //draw camera
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        float length;
        if(i == WINDOW_SIZE - 1)
        {
            length = 0.3;
        }
        else
        {
            length = 0.1;
        }
        vector<Vector3f> camera_coner_w = calculate_camera_pose(T_window[i], R_window[i], length);
        vector<cv::Point2f> camera_coner;
        for(auto it : camera_coner_w)
        {
            camera_coner.push_back(World2VirturCam(it, depth_marker));
        }
        
        CvScalar camera_color = cvScalar(0,0,255);
        cv::line(result, camera_coner[0], camera_coner[1], camera_color, 1, 8, 0);  //RGB
        cv::line(result, camera_coner[1], camera_coner[2], camera_color, 1, 8, 0);
        cv::line(result, camera_coner[2], camera_coner[3], camera_color, 1, 8, 0);
        cv::line(result, camera_coner[3], camera_coner[0], camera_color, 1, 8, 0);
        
        cv::line(result, camera_coner[4], camera_coner[0], camera_color, 1, 8, 0);
        cv::line(result, camera_coner[4], camera_coner[1], camera_color, 1, 8, 0);
        cv::line(result, camera_coner[4], camera_coner[2], camera_color, 1, 8, 0);
        cv::line(result, camera_coner[4], camera_coner[3], camera_color, 1, 8, 0);
    }
    
    //draw existing boxes
    if (box_in_trajectorty)
        drawBoxVirturCam(result);
    
    for (int i=0; i<point_cloud.size(); i++)
    {
        Eigen::Vector3f Pc;
        Pc = RIC.transpose()*point_cloud[i];
        Eigen::Vector3f Pv;
        pts = World2VirturCam(point_cloud[i], depth_marker);
        if(checkBorder(pts))
        {
            cv::circle(result, pts, 0, cvScalar(0,255,0), 3);
        }
    }
    
}

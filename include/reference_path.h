
/**************************************************************************
 * @Description : 
 * @Author      : apple
 * @Email       : sunjundedipan@163.com
 * @Date        : Do not edit
 * @LastEditTime: 2025-05-26 10:06:51
 **************************************************************************/

#pragma once

#include<iostream>
#include<vector>
#include<cmath>
#include<algorithm>
#include<Eigen/Dense>


using namespace std;
using namespace Eigen;


struct parameters
{
    int L;
    int NX, NU, T;
    double dt;
};

struct reference_path
{
    float x, y, yaw, k; // 位置x, 位置y， 轨迹点的切线方向, 曲率k 切线方向和实际航向角可能有个转换关系 M_PI*0.5 - YAW
} way_point_t;


class ReferencePath
{
public:
    ReferencePath();
    vector<double> calc_track_error(vector<double> robot_state);
    double normalize_angle(double angle);

public:
    vector<vector<double>> refer_path; // refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
    vector<double> refer_x, refer_y;

    vector<way_point_t> path;
};




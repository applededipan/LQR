
/**************************************************************************
 * @Description : 
 * @Author      : apple
 * @Email       : sunjundedipan@163.com
 * @Date        : Do not edit
 * @LastEditTime: 2025-05-24 15:46:56
 **************************************************************************/

#pragma once

#include<iostream>
#include<vector>
#include<cmath>
#include<algorithm>
#include<Eigen/Dense>


using namespace std;
using namespace Eigen;

constexpr double PI = 3.1415926;


struct parameters
{
    int L;
    int NX, NU, T;
    double dt;
};


class MyReference_path
{
public:
    MyReference_path();
    vector<double> calc_track_error(vector<double> robot_state);
    double normalize_angle(double angle);

public:
    vector<vector<double>> refer_path; // refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
    vector<double> refer_x, refer_y;
};




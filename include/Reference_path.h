
/**************************************************************************
 * @Description : 
 * @Author      : apple
 * @Email       : sunjundedipan@163.com
 * @Date        : Do not edit
 * @LastEditTime: 2025-05-24 14:31:43
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
    vector<double> calcTrackError( vector<double> robot_state);
    double normalizeAngle(double angle);

public:
    vector<vector<double>> refer_path; // refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
    vector<double> refer_x,refer_y;
};




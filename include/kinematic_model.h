/**************************************************************************
 * @Description : 运动学模型建模
 * @Author      : apple
 * @Email       : sunjundedipan@163.com
 * @Date        : Do not edit
 * @LastEditTime: 2025-05-24 14:48:57
 **************************************************************************/
#pragma once



#include<iostream>
#include<vector>
#include<Eigen/Dense>



using namespace std;
using namespace Eigen;


class KinematicModel
{
public:
    double x, y, psi, v, L, dt;

public:
    KinematicModel(double x, double y, double psi, double v, double l, double dt);
    vector<double>get_state();
    void update_state(double accel, double delta_f);
    vector<MatrixXd> state_space(double ref_delta, double ref_yaw);
};




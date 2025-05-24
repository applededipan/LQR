/**************************************************************************
 * @Description : 
 * @Author      : apple
 * @Email       : sunjundedipan@163.com
 * @Date        : Do not edit
 * @LastEditTime: 2025-05-24 14:53:16
 **************************************************************************/

#pragma once

#include<iostream>
#include<vector>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

constexpr double EPS = 1.0e-4;


// Eigen 库非常强大，支持各种矩阵和向量操作，包括但不限于加法、减法、乘法、求逆、特征值分解、LU 分解等。
// 此外，Eigen 库还提供了固定大小的矩阵类，如 Matrix3d（3x3 矩阵）和 Matrix4f（4x4 矩阵），这些类在编译时具有确定的大小，因此可能提供更好的性能。
class LQRControl
{
private:
    int N;     // 可能是迭代步长

public:
    LQRControl(int n);

    // MatrixXd 是 Eigen 库中的一个模板类，用于创建和操作动态大小的矩阵。
    // 参数 A 和 B 分别代表系统的状态矩阵和控制矩阵。
    // 参数 Q 和 R 分别代表状态权重矩阵和控制权重矩阵。
    // 返回值是解得的Riccati矩阵 P
    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
    double lqrControl(vector<double>robot_state, vector<vector<double>>refer_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
};


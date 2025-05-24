/**************************************************************************
 * @Description : 
 * @Author      : apple
 * @Email       : sunjundedipan@163.com
 * @Date        : Do not edit
 * @LastEditTime: 2025-05-24 15:35:37
 **************************************************************************/


#include "lqr.h"
#include<cmath>


LQRControl::LQRControl(int n):N(n){};

// MatrixXd 是 Eigen 库中的一个模板类，用于创建和操作动态大小的矩阵。
// 参数 A 和 B 分别代表系统的状态矩阵和控制矩阵。
// 参数 Q 和 R 分别代表状态权重矩阵和控制权重矩阵。
// Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
// R矩阵元素变大意味着希望控制输入能够尽可能小,即消耗的能耗更小
// 返回值是解得的Riccati矩阵 P
// 黎卡提方程迭代求解P函数.通过迭代求解Riccati方程，得到最优控制的状态权重矩阵 P
MatrixXd LQRControl::cal_ricatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R)
{
    MatrixXd Qf = Q;
    MatrixXd P_old = Qf;
    MatrixXd P_new;

    for (int i = 0; i < N; i++) {
        P_new = Q + A.transpose() * P_old * A - A.transpose() * P_old * B * (R + B.transpose() * P_old * B).inverse() * B.transpose() * P_old * A;

        if ((P_new-P_old).maxCoeff() < EPS && (P_old-P_new).maxCoeff() < EPS) break;
        P_old= P_new;
    }

    return P_new;
}



// ***************************************************************************//
/**
 * LQR控制器 根据当前状态与参考路径的偏差，计算最优控制量（转向角 delta_f）
 * @param robot_state
 * @param refer_path
 * @param s0
 * @param A
 * @param B
 * @param Q
 * @param R
 * @return
*/
double LQRControl::lqr_control(vector<double>robot_state, vector<vector<double>>refer_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R)
{
    MatrixXd X(3, 1);
    // X = [x_curr - x_ref, y_curr - y_ref, yaw_curr - yaw_ref]^T
    X << robot_state[0] - refer_path[s0][0],
         robot_state[1] - refer_path[s0][1],
         robot_state[2] - refer_path[s0][2]; //  x是当前位置和预瞄点的偏差  x,y,yaw三个偏差

    MatrixXd P = cal_ricatti(A, B, Q, R);     //   3*3  黎卡提方程迭代求解P函数

    MatrixXd K = (R+B.transpose()*P*B).inverse()*B.transpose()*P*A;  // 反馈增益 2*3  K = (R + B^T P B)^{-1} B^T P A

    // 替换为QR分解或SVD求解
    // MatrixXd K = (R + B.transpose()*P*B).ldlt().solve(B.transpose()*P*A);

    MatrixXd u = -K*X;   // 2*1
    return u(1, 0);  // return delta_theta 转向角控制量   航向角差值
}
/**************************************************************************
 * @Description : 
 * @Author      : apple
 * @Email       : sunjundedipan@163.com
 * @Date        : Do not edit
 * @LastEditTime: 2025-05-24 14:50:44
 **************************************************************************/


#include"kinematic_model.h"

/**
 * 机器人运动学模型构造
 * @param x     位置x   状态变量
 * @param y     位置y   状态变量
 * @param psi   偏航角  状态变量
 * @param v     速度    状态变量
 * 
 * @param l     轴距
 * @param dt    采样时间
 */
KinematicModel::KinematicModel(double x, double y, double psi, double v, double l, double dt): x(x), y(y), psi(psi), v(v), L(l), dt(dt) {}




/**
 * 根据控制输入（加速度 accel 和 前轮转向角 delta_f），通过离散化公式更新车辆状态
 * @param accel     加速度
 * @param delta_f   前轮转向角控制量
 */
void KinematicModel::update_state(double accel, double delta_f)
{
    x = x + v* cos(psi)*dt;
    y = y + v* sin(psi)*dt;
    psi = psi + v / L * tan(delta_f)*dt;

    v = v + accel*dt;
}


// ************************* 状态获取 ****************************//
vector<double> KinematicModel::get_state()
{
    return {x , y , psi , v};
}


// ********************* 状态空间方程（离散化） 采样周期为0.1s **************************
// ******************* A , B *********************************
// A矩阵：状态转移矩阵（3x3，关联位置和航向角）
// B矩阵：控制输入矩阵（3x2，关联转向角和加速度）
// 输入参数为参考转向角（ref_delta）和参考航向角（ref_yaw）
vector<MatrixXd> KinematicModel::state_space(double ref_delta , double ref_yaw)
{   
    MatrixXd A(3, 3);
    MatrixXd B(3, 2);

    A << 1.0, 0.0,-v*dt*sin(ref_yaw),
         0.0, 1.0, v*dt*cos(ref_yaw),
         0.0, 0.0, 1.0;

    B << dt*cos(ref_yaw), 0,
         dt*sin(ref_yaw), 0,
         dt*tan(ref_delta)/L, v*dt/(L*cos(ref_delta)*cos(ref_delta));
         
    return {A, B};
}
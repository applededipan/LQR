


#include "reference_path.h"



//************************ 构造函数， 求出参考轨迹点上的曲率等信息 ******************************************//
ReferencePath::ReferencePath()
{
    // refer_path = vector<vector<double>> (1000, vector<double>(4));// 1000个点 ， 每个点有x、y、偏航角yaw和曲率（curvature）

    // 生成参考轨迹, 1000个点
    for (int i = 0; i < 1000; i++) {
        way_point_t point;
        point.x = 0.1*i;    // x坐标
        point.y = 2 * sin(point.x / 3.0) + 2.5 * cos(point.x / 2.0);  // y坐标
        path.push_back(point);

        refer_x.push_back(point.x); 
        refer_y.push_back(point.y); 
    }

    double dx, dy, ddx, ddy;
    for (int i = 0; i < path.size(); i++) {
        if (i == 0) {
            dx  = path.at(i+1).x - path.at(i).x;
            dy  = path.at(i+1).y - path.at(i).y;
            ddx = path.at(2).x - path.at(0).x - 2*path.at(1).x; 
            ddy = path.at(2).y - path.at(0).y - 2*path.at(1).y; // 在计算一阶和二阶导数时，对于路径的起点和终点，使用特殊的差分公式来处理边界条件。
        
        } else if (i == path.size()-1) {
            dx  = path.at(i).x - path.at(i-1).x;
            dy  = path.at(i).y - path.at(i-1).y;
            ddx = path.at(i).x + path.at(i-2).x - 2*path.at(i-1).x;
            ddy = path.at(i).y + path.at(i-2).y - 2*path.at(i-1).y;
        
        } else {
            dx  = path.at(i+1).x - path.at(i).x;// 下一个点减当前点 dx
            dy  = path.at(i+1).y - path.at(i).y;// 下一个点减当前点 dy
            ddx = path.at(i+1).x + path.at(i-1).x - 2*path.at(i).x;// 上一个点+下一个点 - 2*当前点    ddx
            ddy = path.at(i+1).y + path.at(i-1).y - 2*path.at(i).y;// 上一个点+下一个点 - 2*当前点    ddy
        }

        path.at(i).yaw = atan2(dy, dx); // 偏航角 yaw(以x轴为0度,转换为实际的话需要做角度转换M_PI*0.5-YAW)
        path.at(i).k   = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3.0 / 2);// 曲率k计算
        // 计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
        // 参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
    }
}



// ********************************计算跟踪误差 **************************************** //
vector<double> ReferencePath::calc_track_error(vector<double>robot_state)
{   //robot_state : 1 *2 (x , y)
    double x = robot_state[0];
    double y = robot_state[1];

    vector<double>d_x(path.size());
    vector<double>d_y(path.size());
    vector<double>d(path.size());

    for (int i = 0; i < path.size(); i++) {
        d_x[i] = path.at(i).x - x;
        d_y[i] = path.at(i).y - y;
        d[i]   = sqrt(d_x[i]*d_x[i]+d_y[i]*d_y[i]);
    }

    int min_index = min_element(d.begin(), d.end()) - d.begin();  // 使用减法可以得到 int 索引------> 车辆/机器人当前位置和参考轨迹距离最短的点 ： 预瞄点

    double yaw = path.at(min_index).yaw;
    double k   = path.at(min_index).k;
    double angle = normalize_angle(yaw - atan2(d_y[min_index], d_x[min_index]));
    double error = d[min_index];
    if (angle < 0) error *= -1;    //  需要判断 车辆/机器人 当前在参考轨迹的左边还是右边   error 有正负号  ------> 负代表在轨迹左边 ， 正表示在轨迹右边

    return {error, k, yaw, min_index};
}


double ReferencePath::normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2*M_PI;
    }

    while (angle < -M_PI) {
        angle += 2*M_PI;
    }

    return angle;
}


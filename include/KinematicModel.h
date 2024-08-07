# ifndef lg_KinematicModel_H
# define lg_KinematicModel_H
#include<iostream>
#include<vector>
#include<Eigen/Dense>
using namespace std;
using namespace Eigen;


class KinematicModel{

    public:
    double x,y,psi,v,L,dt;


    public:
    KinematicModel(double x, double y, double psi, double v, double l, double dt);

    vector<double>getState();

    void updateState(double accel , double delta_f);

    vector<MatrixXd> stateSpace(double ref_delta , double ref_yaw);

};























#endif
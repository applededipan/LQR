#pragma once 
#include<iostream>



class PID_controller{
    public:
    double Kp , Ki , Kd , target , upper , lower;
    double error , pre_error , sum_error;


    public:
    PID_controller(double Kp, double Ki, double Kd, double target, double upper, double lower);

    void setTarget(double target);

    void setK(double Kp , double Ki , double Kd);

    void setBound(double upper , double lower);

    double calOutput(double state);

    void reset();

    void setSumError(double sum_error);

};
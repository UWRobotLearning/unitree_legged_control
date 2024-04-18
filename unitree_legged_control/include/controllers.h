#ifndef _CONTROLLERS_H_
#define _CONTROLLERS_H_

#include <iostream>


class PID{

    private : 
        double kp;
        double ki;
        double kd;
        double e_;
        double e_i;
        double dt;

    public:
        PID(double _kp, double _ki, double _kd) : kp(_kp), ki(_ki), kd(_kd), e_(0.0), e_i(0.0), dt(0.002){}
        double update(double e){
            double e_d = (e - e_)/dt;
            e_i +=  e*dt;
            e_ = e;
            return kp * e + ki * e_i + kd *e_d;
        };

};
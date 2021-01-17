#ifndef UTILS_H
#define UTILS_H

#include "SMSBL.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>

#define DEG2LIN(x) (((x)*4096/360)+2048) // x from -180 to 180
#define LIN2DEG(x) (((x)-2048)*360/4096) // x from 0 to 4096

class LearnPoint{
public:
    // LearnPoint(){}
    s16 joint[6];        // joint angle. 2048  
    double pos[6];      // global position in mm & RPY in rad
};

void forward(LearnPoint &point);
void inverse(LearnPoint &point, LearnPoint &init);

Eigen::Matrix<double, 6, 1> numerical_inverse(Eigen::Matrix<double, 6, 1> global_pose, Eigen::Matrix<double, 6, 1> init_angle);
Eigen::Matrix<double, 6, 6> jacobian(Eigen::Matrix<double, 3, 1> local_position, Eigen::Matrix<double, 6, 1> angle);
Eigen::Matrix<double, 6, 1> local2global(Eigen::Matrix<double, 3, 1> local_position, Eigen::Matrix<double, 6, 1> motor_angle);
Eigen::Matrix<double, 3, 1> tr2RPY (Eigen::Matrix<double, 3, 3> pose);

#endif
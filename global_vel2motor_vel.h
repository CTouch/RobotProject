#ifndef GLOBAL_VEL2MOTOR_VEL_H
#define GLOBAL_VEL2MOTOR

#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#define pi 3.1415926535898

// Eigen::Matrix<double, 6, 1> Cal_global_vel2motor_vel (Eigen::Matrix<double, 6, 1> motor_angle, double vx , double vy , double vz , double v_roll , double v_pitch , double v_yaw);
Eigen::Matrix<double, 6, 1> Cal_global_vel2motor_vel (Eigen::Matrix<double, 6, 1> global_vel, Eigen::Matrix<double, 6, 1> motor_angle);
Eigen::Matrix<double, 6, 6> J_solve(Eigen::Matrix<double, 6, 1> motor_angle);
Eigen::Matrix<double, 4, 4> T_solve(double a, double alpha, double d, double theta);
Eigen::Matrix<double, 3, 1> rodriguez(Eigen::Matrix<double, 3, 3> pose, Eigen::Matrix<double, 3, 1> k);
Eigen::Matrix<double, 3, 1> tr2RPY(Eigen::Matrix<double, 3, 3> pose);

#endif
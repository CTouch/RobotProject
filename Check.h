#include <iostream>
#include "Eigen/Dense"
#include <math.h>
#include "FeedBack.h"

extern FeedBack feedback[6];

double lb[6]={400,1100,3700,500,2048,-1};
double ub[6]={3600,3000,4700,3600,4000,-1};

#define MAX_LOAD = 

// Eigen::Matrix<double, 6, 1> Check_Safe(Eigen::Matrix<double, 6, 1> motor_angle, Eigen::Matrix<double, 6, 1> motor_vel); 
bool Check_Safe(Eigen::Matrix<double, 6, 1> motor_angle, Eigen::Matrix<double, 6, 1> motor_vel); 

Eigen::Matrix<double, 6, 1> Check_theta(Eigen::Matrix<double, 6, 1> motor_real_angle, Eigen::Matrix<double, 6, 1> motor_angle);
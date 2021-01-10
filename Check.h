#ifndef CHECK_H
#define CHECK_H

#include <iostream>
#include "Eigen/Dense"
#include <math.h>
#include "FeedBack.h"
#include "Utils.h"

extern FeedBack feedback[6];



#define MAX_LOAD 500
#define MAX_VEL 1000

// Eigen::Matrix<double, 6, 1> Check_Safe(Eigen::Matrix<double, 6, 1> motor_angle, Eigen::Matrix<double, 6, 1> motor_vel); 
bool Check_Safe(); 


#endif
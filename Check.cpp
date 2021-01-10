#include "Check.h"

// Eigen::Matrix<double, 6, 1> Check_Safe(Eigen::Matrix<double, 6, 1> motor_angle, Eigen::Matrix<double, 6, 1> motor_vel)

double lb[6]={400,1100,3700,500,2048,-1};
double ub[6]={3600,3000,4700,3600,4000,-1};

bool Check_Safe()
{
	bool ret = false;

	// check load
	for (int i = 0; i < 6; i++)
	{
		if (abs(feedback[i].Load) > MAX_LOAD || abs(feedback[i].Speed) > MAX_VEL)
		{
			ret = true;
			break;
		}
	}

	return ret;
}


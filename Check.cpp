#include "Check.h"

// Eigen::Matrix<double, 6, 1> Check_Safe(Eigen::Matrix<double, 6, 1> motor_angle, Eigen::Matrix<double, 6, 1> motor_vel)

double theta_lb[6]={400,1100,3700,500,2048,-1};
double theta_ub[6]={3600,3000,4700,3600,4000,-1};

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

void Check_Theta(LearnPoint &send_theta)
{
    for (int i = 0; i < 6; i++)
    {
        if (feedback[i].Pos < theta_lb[i])
        {
            send_theta.joint[i] = theta_lb[i];
            std::cout << "Joint " << i << ": Too Small\n";
        }
        else if (feedback[i].Pos > theta_ub[i])
        {
            send_theta.joint[i] = theta_ub[i];
            std::cout << "Joint " << i << ": Too Large\n";
        }
    }
}


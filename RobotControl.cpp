#include "RobotControl.h"

extern FeedBack feedback[6];

int direction[6] = {-1, 1, -1, -1, 1, 1};
int initial[6] = {2048, 2048, 2048, 2048, 2048, 2048};

RobotControl::RobotControl(const char *seritalPort)
{
    if (!sm.begin(1000000, seritalPort))
    {
        std::cout << "Failed to init smsbl motor!" << std::endl;
        return;
    }
    status = Status::SINGLE_JOINT;
    std::cout << "start pushed\n";
    for(int i = 0;i < 6;i++){
        if (i >= 3) sm.unLockEprom(i);
        sm.WheelMode(i);
    }   
}

void RobotControl::SolveGlobalControl(const xbox_map_t &map)
{
    std::cout << "-------------global control------------\n";
    Eigen::Matrix<double, 6, 1> global_speed;
    global_speed << 0, 0, 0, 0, 0, 0;

    if (map.xx > 0)
    {
        global_speed(0, 0) = GLOBAL_VEL;
    }
    else if (map.xx < 0)
    {
        global_speed(0, 0) = -1 * GLOBAL_VEL;
    }

    if (map.yy > 0)
    {
        global_speed(1, 0) = GLOBAL_VEL;
    }
    else if (map.yy < 0)
    {
        global_speed(1, 0) = -1 * GLOBAL_VEL;
    }

    if (map.y && !map.a)
    {
        global_speed(2, 0) = GLOBAL_VEL;
    }
    else if (!map.y && map.a)
    {
        global_speed(2, 0) = -1 * GLOBAL_VEL;
    }

    bool flag_stay = 1;
    for(int i = 0;i < 6;i++){
        if(global_speed(i, 0) > 10 || global_speed(i, 0) < -10){
            flag_stay = 0;
            break;
        }
    }
    if (flag_stay) return;  // 静止状态不需要操作
    Eigen::Matrix<double, 6, 1> motor_angle;

    std::cout << "motor angle (in lin):" << std::endl;
    for (int i = 0; i < 6; i++)
    {
        motor_angle(i, 0) = direction[i] * LIN2RAD(feedback[i].Pos);
        std::cout << feedback[i].Pos << std::endl;
    }
    motor_angle(2, 0) -= motor_angle(1, 0);

    std::cout << "motor angle (in degree):\n"
              << motor_angle << std::endl;

    Eigen::Matrix<double, 6, 1> motor_speed = Cal_global_vel2motor_vel(motor_angle, global_speed);  // 度/s

    LearnPoint tp;
    for (int i = 0; i < 6; i++)
    {
        double spe;
        double T_interval = 0.2;    // 0.2s
        Eigen::Matrix<double, 6, 1> delta_joint_angle;
        for (int i = 0;i < 6;i++)
        {
            delta_joint_angle(i,0) = motor_speed(i,0) * T_interval * direction[i];  // 度
            delta_joint_angle(i,0) = 4096.0 / 360 *(delta_joint_angle(i,0));               // s16 用于发送
        }
        delta_joint_angle(2,0) -= delta_joint_angle(1,0);
        delta_joint_angle(1,0) -= 4;
        delta_joint_angle(2,0) -= 3;
        for (int i = 0;i < 6;i++){
            tp.joint[i] = feedback[i].Pos + delta_joint_angle(i,0) ;        // s16 用于发送
        }
    }
    SetPose(tp);
    // usleep(200000);
}
void RobotControl::SolveXbox(const xbox_map_t &map)
{
    if (map.a == 1 && map.y == 0)
    {
        sm.WriteSpe(2, 400, 100);
    }
    else if (map.a == 0 && map.y == 1)
    {
        sm.WriteSpe(2, -400, 100);
    }
    else
        sm.WriteSpe(2, 0, 100);

    if (map.x == 1 && map.b == 0)
    {
        sm.WriteSpe(0, 400, 100);
    }
    else if (map.x == 0 && map.b == 1)
    {
        sm.WriteSpe(0, -400, 100);
    }
    else
    {
        sm.WriteSpe(0, 0, 100);
        sm.WriteSpe(3, 0, 100);
    }
    if (map.xx > 0)
    {
        sm.WriteSpe(3, 400, 100);
    }
    else if (map.xx < 0)
    {
        sm.WriteSpe(3, -400, 100);
    }
    else
        sm.WriteSpe(3, 0, 100);

    if (map.yy > 0)
    {
        sm.WriteSpe(4, 400, 100);
    }
    else if (map.yy < 0)
    {
        sm.WriteSpe(4, -400, 100);
    }
    else
        sm.WriteSpe(4, 0, 100);

    if (map.lb && !map.rb)
    {
        sm.WriteSpe(1, 400, 100);
        sm.WriteSpe(2, -400, 100);
    }
    else if (!map.lb && map.rb)
    {
        sm.WriteSpe(1, -400, 100);
        sm.WriteSpe(2, 400, 100);
    }
    else
    {
        sm.WriteSpe(1, 0, 100);
    }
}

void RobotControl::SolveXboxThread(RobotControl &robotControl, const xbox_map_t &map)
{
    while (1)
    {
        if (map.start == 1)
        {
            std::cout << "start pushed\n";
            for (int i = 0; i < 6; i++)
            {
                if (i >= 3)
                    robotControl.sm.unLockEprom(i);
                robotControl.sm.writeByte(i, SMSBL_MODE, 0);
            }
            robotControl.status = Status::GLOBAL_CONTROL;
        }
        else if (map.back == 1)
        {
            for (int i = 0; i < 6; i++)
            {
                if (i >= 3)
                    robotControl.sm.unLockEprom(i);
                robotControl.sm.WheelMode(i);
            }
            std::cout << "back pushed\n";
            robotControl.status = Status::SINGLE_JOINT;
        }
        else if (map.home == 1)
        {
            std::cout << "home pushed\n";
            robotControl.AddCurrentPose();
        }
        else if (map.lo == 1)
        {
            std::cout << "lo pushed\n";
            if (robotControl.status == SINGLE_JOINT)
            {
                for (int i = 0; i < 6; i++)
                {
                    if (i >= 3)
                        robotControl.sm.unLockEprom(i);
                    robotControl.sm.writeByte(i, SMSBL_MODE, 0);
                }
            }
            robotControl.status = Status::REPERFORM;
        }

        if (robotControl.status == Status::SINGLE_JOINT)
        {
            // std::cout << "single_joint\n";
            robotControl.SolveXbox(map);
        }
        else if (robotControl.status == Status::GLOBAL_CONTROL)
        {
            std::cout << "Global control\n";
            robotControl.SolveGlobalControl(map);
        }
        else if (robotControl.status == Status::REPERFORM)
        {
            std::cout << "Reperform\n";
            robotControl.RePerformNaive();
        }
        std::chrono::milliseconds dura(200);
        std::this_thread::sleep_for(dura);
    }
}

void RobotControl::SetPose(LearnPoint point)
{
    if (Check_Safe())
    {
        // sm.WheelMode(0);
        // sm.WheelMode(1);
        // sm.WheelMode(2);
        // sm.unLockEprom(3);
        // sm.WheelMode(3);
        // sm.unLockEprom(4);
        // sm.WheelMode(4);
        // sm.unLockEprom(5);
        // sm.WheelMode(5);
        // for (int i = 0; i < 6; i++)
        // {
        //     sm.WriteSpe(i, 0, 100);
        // }
        std::cout << "Unsafe Error" << std::endl;
        // exit(0);
    }
    std::cout << "target:\n";
    for (int j = 0; j < 6; j++)
    {
        std::cout << point.joint[j] << std::endl;
    }
    // sm.SyncWritePosEx(ID, 5, point.joint, Speed, ACC);
    for (;;){
        // Check_Theta(point);
        sm.SyncWritePosEx(ID, 5, point.joint, Speed, ACC);
        usleep(20000);
        bool FinishFlag = 1;
        // if(j % 100000L) std::cout << "In wait loop\n";
        for (int i = 0; i < 6; i++)
        {
            if (abs(feedback[i].Pos - point.joint[i]) > 15)
            {
                FinishFlag = 0;
                break;
            }
        }
        if (FinishFlag)
            break;
    }
}
void RobotControl::RePerformNaive()
{
    // for(int i = 0;i < 6;i++){
    //     if (i >= 3) sm.unLockEprom(i);
    //     sm.writeByte(i, SMSBL_MODE, 0);
    // }

    for (int i = 0; i < PointList.size(); i++)
    {
        std::cout << "target" << i << ":\n";
        for (int j = 0; j < 6; j++)
        {
            std::cout << PointList[i].joint[j] << std::endl;
        }
        SetPose(PointList[i]);
    }
    status = Status::IDLE;
    std::cout << "--------finish reperform naive-------\n";
    // if (status == Status::SINGLE_JOINT)
    //     for(int i = 0;i < 6;i++){
    //         if (i >= 3) sm.unLockEprom(i);
    //         sm.WheelMode(i);
    //     }
}

void RobotControl::AddCurrentPose()
{
    LearnPoint pose;
    for (int i = 0; i < 6; i++)
    {
        pose.joint[i] = feedback[i].Pos;
        std::cout << "Pose " << i << ": " << pose.joint[i] << std::endl;
    }
    std::cout << std::endl;

    // forward here
    forward(pose);

    PointList.push_back(pose);
}

// LearnPoint RobotControl::GetPose(){
//     LearnPoint pose;
//     for(int i = 0;i < 6;i++){
//         pose.joint[i] = feedback[i].Pos;
//     }
//     return pose;
// }

void RobotControl::RePerform0()
{

    for (int i = 0; i < PointList.size()-1; i++)
    {
        std::cout << "target" << i << ":\n";
        for (int j = 0; j < 6; j++)
        {
            std::cout << PointList[i].joint[j] << std::endl;
        }
        SetPose_Interpolate(PointList[i], PointList[i+1]);
    }
}

double theta_lb[6]={400,1100,3700,500,2048,-1};
double theta_ub[6]={3600,3000,4700,3600,4000,-1};

bool RobotControl::Check_Safe()
{
	bool ret = false;

	// check load
	for (int i = 0; i < 6; i++)
	{
		if (abs(feedback[i].Load) > MAX_LOAD)
		{
			ret = true;
			std::cout << "Load Unsafe! Joint" << i << " Load " << feedback[i].Load << std::endl;
		}
		if (abs(feedback[i].Speed) > MAX_VEL)
		{
			ret = true;
			std::cout << "Speed Unsafe! Joint" << i << " Speed " << feedback[i].Speed << std::endl;
		}																			
		if (ret) break;
	}

	return ret;
}

void RobotControl::Check_Theta(LearnPoint &send_theta)
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


void RobotControl::SetPose_Interpolate(LearnPoint curPoint, LearnPoint nextPoint)
{
    if (Check_Safe())
    {
        // sm.WheelMode(0);
        // sm.WheelMode(1);
        // sm.WheelMode(2);
        // sm.unLockEprom(3);
        // sm.WheelMode(3);
        // sm.unLockEprom(4);
        // sm.WheelMode(4);
        // sm.unLockEprom(5);
        // sm.WheelMode(5);
        // for (int i = 0; i < 6; i++)
        // {
        //     sm.WriteSpe(i, 0, 100);
        // }
        std::cout << "Unsafe Error" << std::endl;
        // exit(0);
    }


    // get global interpolate point
    std::vector<LearnPoint> Interpolation_Point;


    for (int i = 0; i < Interpolation_Point.size(); i++)
    {
        Check_Theta(Interpolation_Point[i]);
        sm.SyncWritePosEx(ID, 5, Interpolation_Point[i].joint, Speed, ACC);
        usleep(201001);
    }

    // for (;;)
    // {
    //     Check_Theta(point);
    //     sm.SyncWritePosEx(ID, 5, point.joint, Speed, ACC);
    //     usleep(201001);
    //     bool FinishFlag = 1;
    //     for (int i = 0; i < 6; i++)
    //     {
    //         if (abs(feedback[i].Pos - point.joint[i]) > 10)
    //         {
    //             FinishFlag = 0;
    //             break;
    //         }
    //     }
    //     if (FinishFlag)
    //         break;
    // }
}

std::vector<LearnPoint> RobotControl::Interpolation(LearnPoint curPoint, LearnPoint nextPoint)
{
    std::vector<LearnPoint> Interpolation_Point;
    double dx = (nextPoint.x - curPoint.x);
    double dy = (nextPoint.y - curPoint.y);
    double dz = (nextPoint.z - curPoint.z);
    double lineLen = sqrt(dx*dx + dy*dy + dz*dz);

    double x_step = LEN_STEP * dx / lineLen;
    double y_step = LEN_STEP * dy / lineLen;
    double z_step = LEN_STEP * dz / lineLen;

    int stepNum = floor(lineLen / LEN_STEP);
    LearnPoint point;
    point.x = curPoint.x;
    point.y = curPoint.y;
    point.z = curPoint.z;
    Interpolation_Point.push_back(point);

    for (int i = 1; i < stepNum; i++)
    {
        point.x += x_step;
        point.y += y_step;
        point.z += z_step;

        // inverse here
        inverse(point);
        Interpolation_Point.push_back(point);
    }
    point.x = nextPoint.x;
    point.y = nextPoint.y;
    point.z = nextPoint.z;
    Interpolation_Point.push_back(point);

    return Interpolation_Point;
}

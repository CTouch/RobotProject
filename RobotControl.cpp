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
    // Reset();
    sm.WheelMode(0);
    sm.WheelMode(1);
    sm.WheelMode(2);
    sm.unLockEprom(3);
    sm.WheelMode(3);
    sm.unLockEprom(4);
    sm.WheelMode(4);
    sm.unLockEprom(5);
    sm.WheelMode(5);
};

void RobotControl::SolveGlobalControl(const xbox_map_t &map)
{
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

    Eigen::Matrix<double, 6, 1> motor_angle;
    motor_angle << 0, 0, 0, 0, 0, 0;

    std::cout << "motor angle (in lin):" << std::endl;
    for (int i = 0; i < 6; i++)
    {
        // sm.FeedBack(i);

        motor_angle(i, 0) = direction[i] * LIN2RAD(feedback[i].Pos);
        std::cout << feedback[i].Pos << std::endl;
    }
    // motor_angle(2, 0) = direction[2] * (RAD2LIN(direction[1]*motor_angle(1,0)) + sm.ReadPos(-1));
    motor_angle(2,0) = direction[2] * (RAD2LIN(feedback[2].Pos) + RAD2LIN(feedback[1].Pos));

    std::cout << "motor angle (in degree):\n"
              << motor_angle << std::endl;

    Eigen::Matrix<double, 6, 1> motor_speed = Cal_global_vel2motor_vel(motor_angle, global_speed);

    std::cout << "motor vel (in lin):" << std::endl;
    for (int i = 0; i < 6; i++)
    {
        if (i == 2)
        {
            std::cout << DEG2VEL(direction[2] * motor_speed(2, 0) - motor_speed(1, 0)) << std::endl;
            sm.WriteSpe(i, DEG2VEL(direction[2] * motor_speed(2, 0) - motor_speed(1, 0)), 100);
        }
        else
        {
            std::cout << direction[i] * DEG2VEL(motor_speed(i, 0)) << std::endl;
            sm.WriteSpe(i, direction[i] * DEG2VEL(motor_speed(i, 0)), 100);
        }
    }
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
    }
    else if (!map.lb && map.rb)
    {
        sm.WriteSpe(1, -400, 100);
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
            robotControl.status = Status::GLOBAL_CONTROL;
        }
        else if (map.back == 1)
        {
            std::cout << "back pushed\n";
            robotControl.status = Status::SINGLE_JOINT;
        }
        else if (map.home == 1){
            std::cout << "home pushed\n";
            robotControl.AddCurrentPose();
        }
        else if (map.lo == 1){
            std::cout << "lo pushed\n";
            if (robotControl.status == SINGLE_JOINT){
                for(int i = 0;i < 6;i++){
                    if (i >= 3) robotControl.sm.unLockEprom(i);
                    robotControl.sm.writeByte(i, SMSBL_MODE, 0);
                }
            }
            robotControl.status = Status::REPERFORM;
            
        }

        if (robotControl.status == Status::SINGLE_JOINT){
            // std::cout << "single_joint\n";
            robotControl.SolveXbox(map);
        }
        else if (robotControl.status == Status::GLOBAL_CONTROL){
            std::cout << "Global control\n";
            // robotControl.SolveGlobalControl(map);
        }
        else if (robotControl.status == Status::REPERFORM){
            std::cout << "Reperform\n";
            robotControl.RePerformNaive();
        }
        std::chrono::milliseconds dura(200);
        std::this_thread::sleep_for(dura);
    }
}

void RobotControl::SetPose(LearnPoint point){
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
    for(int j = 0; j < 6;j++){
        std::cout << point.joint[j] << std::endl;
    }
    // sm.SyncWritePosEx(ID, 5, point.joint, Speed, ACC);
    for (;;){
        Check_Theta(point);
        sm.SyncWritePosEx(ID, 5, point.joint, Speed, ACC);
        usleep(201001);
        bool FinishFlag = 1;
        // if(j % 100000L) std::cout << "In wait loop\n";
        for(int i = 0;i < 6;i++){
            if(abs(feedback[i].Pos - point.joint[i]) > 10){
               FinishFlag = 0;
               break;
            }
        }
        if(FinishFlag) break;
    }
}
void RobotControl::RePerformNaive(){
    // for(int i = 0;i < 6;i++){
    //     if (i >= 3) sm.unLockEprom(i);
    //     sm.writeByte(i, SMSBL_MODE, 0);
    // }

    for (int i = 0; i < PointList.size(); i++)
    {
        std::cout << "target" << i << ":\n";
        for(int j = 0; j < 6;j++){
            std::cout << PointList[i].joint[j] << std::endl;
        }
        SetPose(PointList[i]);
    }
    // if (status == Status::SINGLE_JOINT)
    //     for(int i = 0;i < 6;i++){
    //         if (i >= 3) sm.unLockEprom(i);
    //         sm.WheelMode(i);
    //     }
}

void RobotControl::AddCurrentPose(){
    LearnPoint pose;
    for(int i = 0;i < 6;i++){
        pose.joint[i] = feedback[i].Pos;
        std::cout << "Pose " << i << ": " << pose.joint[i] << std::endl;
    }
    std::cout << std::endl;

    PointList.push_back(pose);
}

// LearnPoint RobotControl::GetPose(){
//     LearnPoint pose;
//     for(int i = 0;i < 6;i++){
//         pose.joint[i] = feedback[i].Pos;
//     }
//     return pose;
// }

void RobotControl::RePerform0(){
    for(int i = 0;i < 6;i++){
        if (i >= 3) sm.unLockEprom(i);
        sm.writeByte(i, SMSBL_MODE, 0);
    }
    for (int i = 0;i < PointList.size();i++){
        LearnPoint CurrentPose;
        for(int j = 0;j < 6;j++){
            CurrentPose.joint[j] = feedback[j].Pos;
        }
        // for(int j = 0;j < 0)
    }
}
#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H
#include "SMSBL.h"
#include <iostream>
#include "xbox.h"
#include <thread>
#include "FeedBack.h"
#include "global_vel2motor_vel.h"

extern FeedBack feedback[6];

enum Status{
    SINGLE_JOINT,
    GLOBAL_CONTROL
};

#define GLOBAL_VEL 10      // in mm/s
#define SINGLE_VEL 400
#define RAD2LIN(x) (((x)*4096/360/50)+2048)
#define LIN2RAD(x) (((x)-2048)*50*360/4096)

class RobotControl{
private:
    u8 ID[6] = {0, 1, 2, 3, 4, 5};
    s16 Position[6] = {2048, 2048, 2048, 2048, 2048, 2048};
    u16 Speed[6] = {80, 80, 80, 80, 80, 80};
    u8 ACC[6] = {50, 50, 50, 50, 50, 50};
public:
    SMSBL sm;
    RobotControl(const char * seritalPort);
    Status status = SINGLE_JOINT;
    void Reset()
    {
        for (int i = 0; i < 6; i++)
        {
            Position[i] = 2048;
        }
        sm.SyncWritePosEx(ID, 6, Position, Speed, ACC);
        usleep(8e6);
        std::cout << "Reset" << std::endl;
    }

    void SolveXbox(const xbox_map_t &map);
    static void SolveXboxThread(RobotControl & robotControl,const xbox_map_t &map);
    void SolveGlobalControl(const xbox_map_t &map);
};
#endif
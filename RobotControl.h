#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H
#include "SMSBL.h"
#include <iostream>
#include "xbox.h"
class RobotControl{
private:
    u8 ID[6] = {0, 1, 2, 3, 4, 5};
    s16 Position[6] = {2047, 2047, 2047, 2047, 2047, 2047};
    u16 Speed[6] = {80, 80, 80, 80, 80, 80};
    u8 ACC[6] = {50, 50, 50, 50, 50, 50};
    SMSBL sm;
public:
    RobotControl(const char * seritalPort);
    void Reset()
    {
        for (int i = 0; i < 6; i++)
        {
            Position[i] = 2047;
        }
        sm.SyncWritePosEx(ID, 6, Position, Speed, ACC);
        usleep(5e6);
        std::cout << "Reset" << std::endl;
    }

    void SolveXbox(xbox_map_t map);
};
#endif
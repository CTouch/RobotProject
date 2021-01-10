#include <cstdio>
#include <iostream>
#include "../../../SCServo.h"

#define IDN 6
// #define SYNCWRITE 1

SMSBL sm;
u8 ID[IDN] = {0, 1, 2, 3, 4, 5};
u16 poss[IDN] = {0, 0, 0, 0, 0, 0};
s16 posres[IDN] = {2048, 2048, 2048, 2048, 2048, 2048};
double currentpos[IDN] = {0, 0, 0, 0, 0, 0};
bool ExecuteFlag = true;
//trajectory planning parameter
u16 spd[IDN] = {200, 200, 200, 200, 200, 200};
u8 acc[IDN] = {60, 60, 60, 60, 60, 60};

int main(int argc, char **argv)
{
    if (!sm.begin(1000000, "/dev/ttyUSB0"))
    {
        std::cout << "Failed to init smsbl motor!" << std::endl;
        return 0;
    }

#ifdef SYNCWRITE
    sm.SyncWritePosEx(ID, IDN, posres, spd, acc);
#else // Async Write
    sm.WritePosEx(0, 2048, 200, 60);
    sm.WritePosEx(1, 2048, 200, 60);
    sm.WritePosEx(2, 2048, 200, 60);
    sm.WritePosEx(3, 2048, 200, 60);
    sm.WritePosEx(4, 2048, 200, 60);
    sm.WritePosEx(5, 2048, 200, 60);
#endif

    usleep(10*100000);
    sm.end();
    return 0;
}
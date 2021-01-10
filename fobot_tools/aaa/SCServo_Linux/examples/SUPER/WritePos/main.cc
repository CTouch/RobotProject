/*
以下例子在SMS40BL中测试通过，舵机出厂速度单位为0.732rpm，舵机机运行速度V=80
如果使用的出厂速度单位是0.0146rpm，则速度改为V=4000，延时公式T=[(P1-P0)/V]*1000+[V/(A*100)]*1000
*/

#include <cstdio>
#include <iostream>
#include "../../../SCServo.h"

SMSBL sm;

int main(int argc, char **argv)
{
    bool interactive = false;
    if (argc < 1 + 3)
    {
        interactive = true;
    }

    if (!sm.begin(1000000, "/dev/ttyUSB0"))
    {
        std::cout << "Failed to init smsbl motor!" << std::endl;
        return 0;
    }

    if (interactive)
    {
        std::string input;
        int id, pos, v, a;
        while (true) {
            std::getline(std::cin, input);
            if (sscanf(input.c_str(), "%d %d %d %d", &id, &pos, &v, &a) != EOF) {
                sm.WritePosEx(id, pos, v, a);
            } else {
                std::cout << "Error" << std::endl;
            }
        }
    }
    else
    {
        sm.WritePosEx(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4])); //舵机(ID1)以最高速度V=80(50*80步/秒)，加速度A=100(100*100步/秒^2)，运行至P1=4095位置
    }

    usleep(10 * 100000);
    sm.end();
    return 1;
}
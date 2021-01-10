#include "xbox.h"
#include "SMSBL.h"
#include "RobotControl.h"
#include "FeedBack.h"
extern void MyFeedBack(char * serial_name);
int main(int argc, char **argv)
{
    xbox_map_t map;
    int len, type;
    MyFeedBack(argv[1]);

    memset(&map, 0, sizeof(xbox_map_t));
    xbox_control test_xbox_control("/dev/input/js0");
    RobotControl robotControl(argv[1]);
    std::thread thread_solve_xbox(RobotControl::SolveXboxThread, std::ref(robotControl), std::ref(map));
    while(1)
    {
        len = test_xbox_control.xbox_map_read(&map);
        if (len < 0)
        {
            usleep(10 * 1000);
            continue;
        }

        // printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
        //         map.time, map.a, map.b, map.x, map.y, map.lb, map.rb, map.start, map.back, map.home, map.lo, map.ro,
        //         map.xx, map.yy, map.lx, map.ly, map.rx, map.ry, map.lt, map.rt);
        // robotControl.Reset();
        // robotControl.SolveXbox(robotControl,map);
        fflush(stdout);
    }

    return 0;
}
#include "RobotControl.h"
RobotControl::RobotControl(const char * seritalPort)
{
    if(!sm.begin(1000000,seritalPort)){
        std::cout<< "Failed to init smsbl motor!"<<std::endl;
        return;
    }
    Reset();
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

void RobotControl::SolveGlobalControl(const xbox_map_t &map){
    double vx, vy, vz;
    if (map.xx > 0)
    {
        vx = GLOBAL_VEL;
    }
    else if (map.xx < 0)
    {
        vx = -1 * GLOBAL_VEL;
    }
    else
    {
        vx = 0;
    }
    
    if (map.yy > 0)
    {
        vy = GLOBAL_VEL;
    }
    else if (map.yy < 0)
    {
        vy = -1 * GLOBAL_VEL;
    }
    else
    {
        vy = 0;
    }

    if (map.y && !map.a)
    {
        vz = GLOBAL_VEL;
    }
    else if (!map.y && map.a)
    {
        vz = -1 * GLOBAL_VEL;
    }
    else
    {
        vz = 0;
    }
    
    
    

}
void RobotControl::SolveXbox(const xbox_map_t & map){
    if(map.a == 1 && map.b == 0){
        sm.WriteSpe(2,400,100);
    }
    else if(map.a == 0 && map.b == 1){
        sm.WriteSpe(2,-400,100);
    }
    else sm.WriteSpe(2,0,100);
    
    if(map.x == 1 && map.y == 0){
        sm.WriteSpe(0,400,100);
        sm.WriteSpe(3,400,100);
    }
    else if(map.x == 0 && map.y == 1){
        sm.WriteSpe(0,-400,100);
        sm.WriteSpe(3,-400,100);
    }
    else {
        sm.WriteSpe(0,0,100);
        sm.WriteSpe(3,0,100);
    }
    if(map.xx > 0){
        sm.WriteSpe(3,400,100);
    }
    else if(map.xx < 0){
        sm.WriteSpe(3,-400,100);
    }
    else sm.WriteSpe(3,0,100);

    if(map.yy > 0){
        sm.WriteSpe(4,400,100);
    }
    else if(map.yy < 0){
        sm.WriteSpe(4,-400,100);
    }
    else sm.WriteSpe(4,0,100);

    if(map.lb && !map.rb)
    {
        sm.WriteSpe(1,400,100);
    }
    else if(!map.lb && map.rb)
    {
        sm.WriteSpe(1,-400,100);
    }
    else
    {
        sm.WriteSpe(1,0,100);
    }

}

void RobotControl::SolveXboxThread(RobotControl & robotControl, const xbox_map_t & map){
    while(1){
        if (map.start == 1){
            robotControl.status = Status::GLOBAL_CONTROL;

        }
        else if (map.back == 1){
            robotControl.status = Status::SINGLE_JOINT;
        }
        if (robotControl.status == Status::SINGLE_JOINT) robotControl.SolveXbox(map);
        else if(robotControl.status = Status::GLOBAL_CONTROL) robotControl.SolveGlobalControl(map);
        std::chrono::milliseconds dura( 200 );
        std::this_thread::sleep_for( dura );
    }

}

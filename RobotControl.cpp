#include"RobotControl.h"
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

void RobotControl::SolveXbox(xbox_map_t map){
    if(map.a == 1 && map.b == 0){
        sm.WriteSpe(2,160,100);
    }
    else if(map.a == 0 && map.b == 1){
        sm.WriteSpe(2,-160,100);
    }
    else sm.WriteSpe(2,0,100);
    
    if(map.x == 1 && map.y == 0){
        sm.WriteSpe(0,160,100);
        sm.WriteSpe(3,160,100);
    }
    else if(map.x == 0 && map.y == 1){
        sm.WriteSpe(0,-160,100);
        sm.WriteSpe(3,-160,100);
    }
    else {
        sm.WriteSpe(0,0,100);
        sm.WriteSpe(3,0,100);
    }
    if(map.xx > 0){
        sm.WriteSpe(3,160,100);
    }
    else if(map.xx < 0){
        sm.WriteSpe(3,-160,100);
    }
    else sm.WriteSpe(3,0,100);

    if(map.yy > 0){
        sm.WriteSpe(4,160,100);
    }
    else if(map.yy < 0){
        sm.WriteSpe(4,-160,100);
    }
    else sm.WriteSpe(4,0,100);

    if(map.lb && !map.rb)
    {
        sm.WriteSpe(1,160,100);
    }
    else if(!map.lb && map.rb)
    {
        sm.WriteSpe(1,-160,100);
    }
    else
    {
        sm.WriteSpe(1,0,100);
    }
    
}
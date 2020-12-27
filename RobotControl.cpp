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

}
void RobotControl::SolveXbox(const xbox_map_t & map){
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
        // std::chrono::milliseconds dura( 2000 );
        // if(map.a == 1 && map.b == 0){
        //     robotControl.sm.WriteSpe(2,80,100);
        // }
        // else if(map.a == 0 && map.b == 1){
        //     robotControl.sm.WriteSpe(2,-80,100);
        // }
        // else robotControl.sm.WriteSpe(2,0,100);
        
        // if(map.x == 1 && map.y == 0){
        //     robotControl.sm.WriteSpe(0,80,100);
        // }
        // else if(map.x == 0 && map.y == 1){
        //     robotControl.sm.WriteSpe(0,-80,100);
        // }
        // else robotControl.sm.WriteSpe(0,0,100);
        
        // if(map.xx == 32767){
        //     robotControl.sm.WriteSpe(3,80,100);
        // }
        // else if(map.xx == -32767){
        //     robotControl.sm.WriteSpe(3,-80,100);
        // }
        // else robotControl.sm.WriteSpe(3,0,100);

        // if(map.yy == 32767){
        //     robotControl.sm.WriteSpe(4,80,100);
        // }
        // else if(map.yy == -32767){
        //     robotControl.sm.WriteSpe(4,-80,100);
        // }
        // else robotControl.sm.WriteSpe(4,0,100);
    }

}

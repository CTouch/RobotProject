#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H
#include "SMSBL.h"
#include <iostream>
#include "xbox.h"
#include <thread>
#include "FeedBack.h"
#include "global_vel2motor_vel.h"
#include "math.h"
#include <vector>
#include "Check.h"

extern FeedBack feedback[6];

enum Status{
    SINGLE_JOINT,
    GLOBAL_CONTROL,
    REPERFORM
};

#define GLOBAL_VEL 10  // in mm/s
#define SINGLE_VEL 400
#define THRESHOLD 5
#define RAD2LIN(x) (((x)*4096/360)+2048)
#define LIN2RAD(x) (((x)-2048)*360/4096)
#define DEG2VEL(x) ((x)*4096/360)
#define VEL2DEG(x) ((x)*360/4096)

class LearnPoint{
public:
    // LearnPoint(){}
    s16 joint[6];        // joint angle. 2048  
};
class RobotControl{
private:
    u8 ID[6] = {0, 1, 2, 3, 4, 5};
    s16 Position[6] = {2048, 2048, 2048, 2048, 2048, 2048};
    u16 Speed[6] = {80, 80, 80, 80, 80, 80};
    u8 ACC[6] = {50, 50, 50, 50, 50, 50};
    std::vector<LearnPoint> PointList;
public:
    SMSBL sm;
    RobotControl(const char * seritalPort);
    Status status = SINGLE_JOINT;
    void Reset();

    void SolveXbox(const xbox_map_t &map);
    static void SolveXboxThread(RobotControl & robotControl,const xbox_map_t &map);
    void SolveGlobalControl(const xbox_map_t &map);

    void ClearPointList(){PointList.clear();}
    void AddtoPointList(LearnPoint point){PointList.push_back(point);}
    
    // 添加当前六关节角度
    void AddCurrentPose();

    //// 获得当前6角度
    // LearnPoint GetPose();       
    void SetPose(LearnPoint point);

    void RePerformNaive();
};
#endif
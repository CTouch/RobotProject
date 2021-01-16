#ifndef UTILS_H
#define UTILS_H

#include "SMSBL.h"

class LearnPoint{
public:
    // LearnPoint(){}
    s16 joint[6];        // joint angle. 2048  
    double x;       // position in global
    double y;
    double z;
};

void forward(LearnPoint &point);
void inverse(LearnPoint &point);

#endif
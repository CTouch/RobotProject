#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <iostream>
#include "SCServo.h"
#include <thread>
#include <pthread.h>  


struct FeedBack
{
	int Joint;
	int Pos;
	int Speed;
	int Load;
	int Temper;
	int Move;
};

void MyFeedBack(int argc, char **argv);
void update_data();
void refresh_timer();

#endif
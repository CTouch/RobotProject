#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <iostream>
#include "SCServo.h"
#include <thread>
#include <pthread.h>  

class MyFeedBack
{
private:
	SMSBL sm;
public:
	struct JointInfo{
		int Joint;
		int Pos;
		int Speed;
		int Load;
		int Temper;
		int Move;
	};
	JointInfo jointinfo[6];
	// void MyFeedBack(char * serial_name);
	void update_data();
	void refresh_timer();
	MyFeedBack(char * serial_name);
};


#endif
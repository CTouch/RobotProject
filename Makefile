.PHONY:build
build:Main

Main:Main.o xbox.o SMSBL.o SCSerial.o SCS.o RobotControl.o FeedBack.o global_vel2motor_vel.o
	g++  -o Main Main.o xbox.o SMSBL.o SCSerial.o SCS.o global_vel2motor_vel.o RobotControl.o FeedBack.o -lpthread 


Main.o:Main.cpp FeedBack.cpp
	g++ -c Main.cpp FeedBack.cpp

FeedBack.o:FeedBack.cpp
	g++ -c FeedBack.cpp

global_vel2motor_vel.o:global_vel2motor_vel.cpp global_vel2motor_vel.h
	g++ -c global_vel2motor_vel.cpp global_vel2motor_vel.h

RobotControl.o:RobotControl.cpp
	g++ -c RobotControl.cpp
SCS.o:SCS.h SCS.cpp
	g++ -c SCS.cpp
SMSBL.o:SMSBL.h SMSBL.cpp
	g++ -c SMSBL.cpp

SCSerial.o:SCSerial.h SCSerial.cpp
	g++ -c SCSerial.cpp
xbox.o:xbox.cpp xbox.h
	g++ -c xbox.cpp xbox.h


.PHONY:clean
clean:
	-rm Main
	-rm *.o
	-rm *.gch
.PHONY:build
build:Main
PROJECT := Main
SRCS    := $(wildcard *.cpp)
OBJS    := $(SRCS:.cpp=.o)
CXX     := g++

$(PROJECT): $(OBJS)
	$(CXX) -o $@ $^ -lpthread

.cpp.o:
	$(CXX) -c $< -o $@

# Main:Main.o xbox.o SMSBL.o SCSerial.o SCS.o RobotControl.o FeedBack.o global_vel2motor_vel.o Check.o
# # 	g++  -o Main Main.o xbox.o SMSBL.o SCSerial.o SCS.o RobotControl.o FeedBack.o global_vel2motor_vel.o Check.o -lpthread 

# Main.o:Main.cpp FeedBack.cpp
# 	g++ -c Main.cpp FeedBack.cpp

# FeedBack.o:FeedBack.cpp
# 	g++ -c FeedBack.cpp

# Check.o:Check.cpp
# 	g++ -c Check.cpp

# global_vel2motor_vel.o:global_vel2motor_vel.cpp 
# 	g++ -c global_vel2motor_vel.cpp 

# RobotControl.o:RobotControl.cpp
# 	g++ -c RobotControl.cpp

# SCS.o:SCS.h SCS.cpp
# 	g++ -c SCS.cpp
# SMSBL.o:SMSBL.h SMSBL.cpp
# 	g++ -c SMSBL.cpp

# SCSerial.o:SCSerial.h SCSerial.cpp
# 	g++ -c SCSerial.cpp
# xbox.o:xbox.cpp xbox.h
# 	g++ -c xbox.cpp xbox.h

.PHONY:clean
clean:
	-rm Main
	-rm *.o
	-rm *.gch
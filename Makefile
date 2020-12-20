.PHONY:build
build:Main

Main:Main.o xbox.o SMSBL.o SCSerial.o SCS.o RobotControl.o
	g++ -o Main Main.o xbox.o SMSBL.o SCSerial.o SCS.o RobotControl.o


Main.o:Main.cpp
	g++ -c Main.cpp

RobotControl.o:RobotControl.o
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
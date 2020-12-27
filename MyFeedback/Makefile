.PHONY:build
build:Main

Main:Main.o FeedBack.o SMSBL.o SCSerial.o SCS.o 
	g++ -o Main Main.o FeedBack.o SMSBL.o SCSerial.o SCS.o -lpthread

Main.o:Main.cpp
	g++ -c Main.cpp

FeedBack.o:FeedBack.h FeedBack.cpp
	g++ -c FeedBack.cpp -lpthread


SCS.o:SCS.h SCS.cpp
	g++ -c SCS.cpp
SMSBL.o:SMSBL.h SMSBL.cpp
	g++ -c SMSBL.cpp

SCSerial.o:SCSerial.h SCSerial.cpp
	g++ -c SCSerial.cpp

.PHONY:clean
clean:
	-rm FeedBack
	-rm *.o
CFLAGS=-I/usr/include/python2.7 -I../
LIBS=-lstdc++ -lbluetooth -lpthread -lrt 

all:_mobot.so

mobot_wrap.c:../mobot.h 
	swig -python -c++ -o mobot_wrap.c ../mobot.h 

mobot_wrap.o:mobot_wrap.c ../mobot.h
	g++ -c mobot_wrap.c $(CFLAGS) -o mobot_wrap.o

mobot.o:../mobot.cpp ../mobot.h
	g++ -c ../mobot.cpp -o mobot.o

_mobot.so:mobot.o mobot_wrap.o
	g++ -shared mobot.o mobot_wrap.o $(LIBS) -o _mobot.so

clean:
	rm -rf mobot_wrap.c mobot.o mobot_wrap.o _mobot.so mobot.py mobot.pyc

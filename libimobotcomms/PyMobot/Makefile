CFLAGS=-I/usr/include/python2.7 -I../ -I../BaroboConfigFile -I../BaroboConfigFile/mxml-2.7 -fPIC
LIBS=-L../BaroboConfigFile -L../BaroboConfigFile/mxml-2.7 -lbaroboconfigfile -lmxml -lstdc++ -lbluetooth -lpthread -lrt 

SRCS=../mobot.c \
../mobot++.cpp \
../mobot_get_functions.c \
../mobot_get_functions++.cpp \
../mobotgroup++.cpp \
../moboti++.cpp \
../moboti_get_functions++.cpp \
../mobotigroup++.cpp \
../moboti_motion_functions++.cpp \
../moboti_movement_functions++.cpp \
../moboti_record_functions++.cpp \
../moboti_set_functions++.cpp \
../mobotl++.cpp \
../mobotl_get_functions++.cpp \
../mobotlgroup++.cpp \
../mobotl_motion_functions++.cpp \
../mobotl_movement_functions++.cpp \
../mobotl_record_functions++.cpp \
../mobotl_set_functions++.cpp \
../mobot_motion_functions.c \
../mobot_motion_functions++.cpp \
../mobot_movement_functions.c \
../mobot_movement_functions++.cpp \
../mobot_record_functions.c \
../mobot_record_functions++.cpp \
../mobot_set_functions.c \
../mobot_set_functions++.cpp 

OBJS=mobot.o \
mobot++.o \
mobot_get_functions.o \
mobot_get_functions++.o \
mobotgroup++.o \
moboti++.o \
moboti_get_functions++.o \
mobotigroup++.o \
moboti_motion_functions++.o \
moboti_movement_functions++.o \
moboti_record_functions++.o \
moboti_set_functions++.o \
mobotl++.o \
mobotl_get_functions++.o \
mobotlgroup++.o \
mobotl_motion_functions++.o \
mobotl_movement_functions++.o \
mobotl_record_functions++.o \
mobotl_set_functions++.o \
mobot_motion_functions.o \
mobot_motion_functions++.o \
mobot_movement_functions.o \
mobot_movement_functions++.o \
mobot_record_functions.o \
mobot_record_functions++.o \
mobot_set_functions.o \
mobot_set_functions++.o 

all:_mobot.so

mobot_wrap.c:../mobot.h 
	swig -python -c++ -o mobot_wrap.c ../mobot.h 

mobot_wrap.o:mobot_wrap.c ../mobot.h
	g++ -c mobot_wrap.c $(CFLAGS) -o mobot_wrap.o

#mobot.o:../mobot.cpp ../mobot.h
#	g++ -c -DNONRELEASE ../mobot.cpp -o mobot.o

%.o:../%.c
	g++ -c -DNONRELEASE  $(CFLAGS) $< -o $@

%.o:../%.cpp
	g++ -c -DNONRELEASE  $(CFLAGS) $< -o $@

_mobot.so:$(OBJS) mobot_wrap.o
	g++ -shared mobot_wrap.o $(OBJS) $(LIBS) -o _mobot.so

clean:
	rm -rf mobot_wrap.c *.o _mobot.so mobot.py mobot.pyc

HEADERS=../mobot.h

target: libmobot.dl

libmobot.dl: mobot.o chmobot_chdl.o
	ch dllink libmobot.dl cplusplus mobot.o chmobot_chdl.o

mobot.o: ../mobot.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus ../mobot.cpp

chmobot_chdl.o: chmobot_chdl.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus chmobot_chdl.cpp

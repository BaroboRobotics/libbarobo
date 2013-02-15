PACKAGE=chmobot
VERSION=0.1
HEADERS=../mobot.h
PKGDIR=$(PACKAGE)-$(VERSION)/$(PACKAGE)

target: libmobot.dl

libmobot.dl: mobot.o chmobot_chdl.o mobot_get_functions.o mobot_motion_functions.o mobot_movement_functions.o mobot_record_functions.o mobot_set_functions.o
	ch dllink libmobot.dl cplusplus mobot.o chmobot_chdl.o -L../../../BaroboConfigFile -lbaroboconfigfile -lbluetooth -lpthread -lrt -lmxml

mobot.o: ../mobot.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus -DNONRELEASE -I../../../BaroboConfigFile ../mobot.cpp

mobot_get_functions.o: ../mobot_get_functions.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus -DNONRELEASE -I../../../BaroboConfigFile ../mobot_get_functions.cpp

mobot_motion_functions.o: ../mobot_motion_functions.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus -DNONRELEASE -I../../../BaroboConfigFile ../mobot_motion_functions.cpp

mobot_movement_functions.o: ../mobot_movement_functions.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus -DNONRELEASE -I../../../BaroboConfigFile ../mobot_movement_functions.cpp

mobot_record_functions.o: ../mobot_record_functions.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus -DNONRELEASE -I../../../BaroboConfigFile ../mobot_record_functions.cpp

mobot_set_functions.o: ../mobot_set_functions.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus -DNONRELEASE -I../../../BaroboConfigFile ../mobot_set_functions.cpp

chmobot_chdl.o: chmobot_chdl.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus chmobot_chdl.cpp

install:
	ch pkginstall.ch $(PACKAGE)

uninstall:
	ch pkginstall.ch -u $(PACKAGE)

createpkg: libmobot.dl
	echo Building $(PACKAGE)-$(VERSION).zip ...
	rm -rf $(PACKAGE)-$(VERSION)
	rm -rf $(PACKAGE)-$(VERSION).zip
	mkdir -p $(PKGDIR)
	mkdir $(PKGDIR)/lib
	mkdir $(PKGDIR)/demos
	mkdir $(PKGDIR)/dl
	mkdir $(PKGDIR)/include
	mkdir $(PKGDIR)/docs
	cp chmobot.chf $(PKGDIR)/lib
	cp shiftTime.chf $(PKGDIR)/lib
	cp delay.chf $(PKGDIR)/lib
	cp libmobot.dl $(PKGDIR)/dl
	cp ../mobot.h $(PKGDIR)/include
	cp ../docs/mobot.pdf $(PKGDIR)/docs
	cp -R ../demos/chdemos/* $(PKGDIR)/demos
	cp Makefile $(PACKAGE)-$(VERSION)
	cp Readme.txt $(PACKAGE)-$(VERSION)
	zip -rq $(PACKAGE)-$(VERSION).zip $(PACKAGE)-$(VERSION)
	

clean:
	rm *.o *.dl



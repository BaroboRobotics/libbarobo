PACKAGE=chimobot
VERSION=0.1
HEADERS=../imobot.h
PKGDIR=$(PACKAGE)-$(VERSION)/$(PACKAGE)
LFLAGS=-L../ -L../libi2c
LIBS=-lbluetooth -lpthread -limobot -li2c

target: libimobot.dl

libimobot.dl: chimobot_chdl.o
	ch dllink libimobot.dl cplusplus chimobot_chdl.o $(LFLAGS) $(LIBS)

imobot.o: ../imobot.cpp $(HEADERS)
	ch dlcomp libimobot.dl cplusplus ../imobot.cpp

chimobot_chdl.o: chimobot_chdl.cpp $(HEADERS)
	ch dlcomp libimobot.dl cplusplus chimobot_chdl.cpp

install:
	ch pkginstall.ch $(PACKAGE)

uninstall:
	ch pkginstall.ch -u $(PACKAGE)

createpkg: libimobot.dl
	echo Building $(PACKAGE)-$(VERSION).zip ...
	rm -rf $(PACKAGE)-$(VERSION)
	rm -rf $(PACKAGE)-$(VERSION).zip
	mkdir -p $(PKGDIR)
	mkdir $(PKGDIR)/lib
	mkdir $(PKGDIR)/demos
	mkdir $(PKGDIR)/dl
	mkdir $(PKGDIR)/include
	mkdir $(PKGDIR)/docs
	cp chimobot.chf $(PKGDIR)/lib
	cp libimobot.dl $(PKGDIR)/dl
	cp ../imobot.h $(PKGDIR)/include
	#cp ../docs/imobot.pdf $(PKGDIR)/docs
	#cp ../demos/gettingStarted/gettingStarted.ch $(PKGDIR)/demos
	#cp ../demos/inchworm/inchworm.ch $(PKGDIR)/demos
	#cp ../demos/inchworm/inchworm2.ch $(PKGDIR)/demos
	cp ../../demos/simple_cpp/simple.ch $(PKGDIR)/demos
	#cp ../demos/stand/stand.ch $(PKGDIR)/demos
	#cp ../demos/stand/stand2.ch $(PKGDIR)/demos
	#cp ../demos/multipleModules/multipleModules.ch $(PKGDIR)/demos
	cp Makefile $(PACKAGE)-$(VERSION)
	cp Readme.txt $(PACKAGE)-$(VERSION)
	zip -rq $(PACKAGE)-$(VERSION).zip $(PACKAGE)-$(VERSION)
	

clean:
	rm *.o *.dl



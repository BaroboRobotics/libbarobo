PACKAGE=chmobot
VERSION=0.1.0
HEADERS=../mobot.h
PKGDIR=$(PACKAGE)-$(VERSION)/$(PACKAGE)

target: libmobot.dl

libmobot.dl: mobot.o chmobot_chdl.o
	ch dllink libmobot.dl cplusplus mobot.o chmobot_chdl.o -lbluetooth -lpthread

mobot.o: ../mobot.cpp $(HEADERS)
	ch dlcomp libmobot.dl cplusplus ../mobot.cpp

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
	cp libmobot.dl $(PKGDIR)/dl
	cp ../mobot.h $(PKGDIR)/include
	cp ../docs/mobot.pdf $(PKGDIR)/docs
	cp ../demos/gettingStarted/gettingStarted.ch $(PKGDIR)/demos
	cp ../demos/inchworm/inchworm.ch $(PKGDIR)/demos
	cp ../demos/simple/simple.ch $(PKGDIR)/demos
	cp ../demos/stand/stand.ch $(PKGDIR)/demos
	cp ../demos/stand/stand2.ch $(PKGDIR)/demos
	cp ../demos/multipleModules/multipleModules.ch $(PKGDIR)/demos
	cp Makefile $(PACKAGE)-$(VERSION)
	cp Readme.txt $(PACKAGE)-$(VERSION)
	zip -rq $(PACKAGE)-$(VERSION).zip $(PACKAGE)-$(VERSION)
	

clean:
	rm *.o *.dl



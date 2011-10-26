#CC=arm-angstrom-linux-gnueabi-gcc 
CC=gcc

CFLAGS=-c -fPIC -I/usr/local/ch/extern/include
# Gumstix LFLAG
#LFLAGS=-u__builtin_Ch_VaArg -u__builtin_Ch_VaStart -u str2ba -L../../gumstix_ch/ch/extern/lib/ -lchsdk -lbluetooth -shared -rdynamic -Wl,-soname,libchbluetooth.so.1 
LFLAGS=-u__builtin_Ch_VaArg -u__builtin_Ch_VaStart -u str2ba -L/usr/local/ch/extern/lib -lchsdk -lbluetooth -shared -rdynamic -Wl,-soname,libchbluetooth.so.1 

all: bluetooth_chdl.dl 

bluetooth_chdl.o:bluetooth_chdl.c
	$(CC) $(CFLAGS) -o bluetooth_chdl.o bluetooth_chdl.c 

bluetooth_chdl.dl:bluetooth_chdl.o
	$(CC) $(LFLAGS) -o libbluetooth_chdl.dl bluetooth_chdl.o

clean:
	rm -rf *.o *.dl

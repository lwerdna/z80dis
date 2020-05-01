# some tests compare against z80ex and libopcodes
#
# libopcodes instructions:
# - get binutils 2.30 and decompress.
# - enter the libiberty directory, ./configure && make then cp libiberty.a /usr/local/lib
# - enter the bfd directory, edit configure and set all_targets=true then ./configure && make && make install
# - enter the opcodes directory, edit configure and set all_targets=true then ./configure && make && make install
#
# z80ex instructions:
# - get z80ex 1.1.21 https://sourceforge.net/projects/z80ex/files/latest/download and decompress
# - enter the directory and `make`

CPPFLAGS = 
CPPFLAGS += -I/usr/local/include
CPPFLAGS += -I/$(HOME)/Downloads/z80ex/include
CPPFLAGS += -std=c++11 -O0 -g

.PHONY: package pypitest

all: gofer.so

gofer.so: gofer.cpp
	g++ $(CPPFLAGS) \
		-lbfd -liberty -lopcodes -lz \
		-shared -o gofer.so gofer.cpp \
		$(HOME)/Downloads/z80ex-1.1.21/z80ex_dasm.o \
		-I$(HOME)/Downloads/z80ex-1.1.21/include \
		-Wl,-headerpad_max_install_names

clean:
	rm -f gofer.so
	rm -rf gofer.so.dSYM


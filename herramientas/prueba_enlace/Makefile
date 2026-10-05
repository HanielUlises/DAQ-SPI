# Compila prueba_enlace en Linux con libftdi1 (Fedora: libftdi-devel,
# Debian/Ubuntu: libftdi1-dev). En Windows se usa compilar.bat.

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra
FTDI_CFLAGS ?= $(shell pkg-config --cflags libftdi1)
FTDI_LIBS   ?= $(shell pkg-config --libs libftdi1)

prueba_enlace: prueba_enlace.cpp ../comun/enlace_prueba.h ../comun/mpsse_linux.h \
               ../../protocolo/daq_protocolo.h
	$(CXX) $(CXXFLAGS) $(FTDI_CFLAGS) -o $@ prueba_enlace.cpp $(FTDI_LIBS)

.PHONY: clean
clean:
	rm -f prueba_enlace

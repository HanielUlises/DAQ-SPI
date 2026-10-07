# Arnés que ejecuta las S-Functions de SIMULINK/DaqPic sin MATLAB, en Linux,
# contra la tarjeta (libftdi1, como prueba_enlace). simulacion/ imita las
# partes de simstruc.h, windows.h y libMPSSE-SPI que usan los bloques.
#
#   make            compila
#   make correr     compila y ejecuta (requiere la tarjeta y el FT2232H libre)
#
# Las fuentes se copian a obj/: SIMULINK/DaqPic contiene los ftd2xx.h y
# libmpsse_spi.h de Windows, que el compilador tomaría antes que los de
# simulacion/ por estar junto a los .cpp.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter
FTDI_CFLAGS ?= $(shell pkg-config --cflags libftdi1)
FTDI_LIBS   ?= $(shell pkg-config --libs libftdi1)

BLOQUES = daqPic daqPicInicio daqPicEscribir daqPicLeer initializePic escribirPic leerPic
FUENTES = $(addprefix ../../SIMULINK/DaqPic/,$(addsuffix .cpp,$(BLOQUES)) daq_bloques.h)
SIM     = $(wildcard simulacion/*.h)
OBJS    = $(addprefix obj/,$(addsuffix .o,$(BLOQUES))) obj/arnes.o

arnes: $(OBJS)
	$(CXX) -o $@ $(OBJS) $(FTDI_LIBS)

obj/SIMULINK/DaqPic/%.cpp: ../../SIMULINK/DaqPic/%.cpp
	@mkdir -p obj/SIMULINK/DaqPic
	@ln -sfn ../../../protocolo obj/protocolo
	cp $< $@

obj/SIMULINK/DaqPic/daq_bloques.h: ../../SIMULINK/DaqPic/daq_bloques.h
	@mkdir -p obj/SIMULINK/DaqPic
	cp $< $@

obj/%.o: obj/SIMULINK/DaqPic/%.cpp obj/SIMULINK/DaqPic/daq_bloques.h $(SIM) \
         ../comun/mpsse_linux.h ../../protocolo/daq_protocolo.h
	$(CXX) $(CXXFLAGS) -Isimulacion $(FTDI_CFLAGS) -c $< -o $@

# Los bloques con los nombres anteriores incluyen el .cpp del bloque nuevo
obj/initializePic.o: obj/SIMULINK/DaqPic/daqPicInicio.cpp
obj/escribirPic.o: obj/SIMULINK/DaqPic/daqPicEscribir.cpp
obj/leerPic.o: obj/SIMULINK/DaqPic/daqPicLeer.cpp

obj/arnes.o: arnes.cpp $(SIM) ../../SIMULINK/DaqPic/daq_bloques.h
	@mkdir -p obj
	$(CXX) $(CXXFLAGS) -Isimulacion $(FTDI_CFLAGS) -c $< -o $@

# Las copias de obj/ se conservan entre compilaciones
.SECONDARY:

.PHONY: correr clean
correr: arnes
	./arnes

clean:
	rm -rf obj arnes

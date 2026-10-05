# Compila visor_enlace en Linux. Requiere libftdi1 y GLFW 3 con OpenGL
# (Fedora: libftdi-devel glfw-devel mesa-libGL-devel; Debian/Ubuntu:
# libftdi1-dev libglfw3-dev libgl-dev). Dear ImGui e ImPlot están en
# terceros/ y se compilan aquí.
#
#   make            compila visor_enlace
#   make prueba     compila y ejecuta las pruebas de bits.h (sin hardware)

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
FTDI_CFLAGS ?= $(shell pkg-config --cflags libftdi1)
FTDI_LIBS   ?= $(shell pkg-config --libs libftdi1)
GLFW_CFLAGS ?= $(shell pkg-config --cflags glfw3)
GLFW_LIBS   ?= $(shell pkg-config --libs glfw3) -lGL

IMGUI  = terceros/imgui
IMPLOT = terceros/implot
INCLUDES = -I$(IMGUI) -I$(IMPLOT)

TERCEROS = $(IMGUI)/imgui.cpp $(IMGUI)/imgui_draw.cpp $(IMGUI)/imgui_tables.cpp \
           $(IMGUI)/imgui_widgets.cpp $(IMGUI)/backends/imgui_impl_glfw.cpp \
           $(IMGUI)/backends/imgui_impl_opengl3.cpp $(IMPLOT)/implot.cpp $(IMPLOT)/implot_items.cpp
OBJ_TERCEROS = $(patsubst %.cpp,obj/%.o,$(notdir $(TERCEROS)))

ENCABEZADOS = adquisicion.h bits.h simulador.h ../comun/enlace_prueba.h \
              ../comun/mpsse_linux.h ../../protocolo/daq_protocolo.h

vpath %.cpp $(IMGUI) $(IMGUI)/backends $(IMPLOT)

visor_enlace: obj/visor_enlace.o obj/adquisicion.o $(OBJ_TERCEROS)
	$(CXX) -o $@ $^ $(FTDI_LIBS) $(GLFW_LIBS) -lpthread

obj/visor_enlace.o: visor_enlace.cpp $(ENCABEZADOS) | obj
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(FTDI_CFLAGS) $(GLFW_CFLAGS) -c -o $@ $<

obj/adquisicion.o: adquisicion.cpp $(ENCABEZADOS) | obj
	$(CXX) $(CXXFLAGS) $(FTDI_CFLAGS) -c -o $@ $<

# Código de terceros: sin advertencias adicionales
obj/%.o: %.cpp | obj
	$(CXX) -std=c++17 -O2 $(INCLUDES) $(GLFW_CFLAGS) -c -o $@ $<

obj:
	mkdir -p obj

prueba_bits: prueba_bits.cpp bits.h simulador.h ../comun/enlace_prueba.h
	$(CXX) -std=c++11 -O2 -Wall -Wextra $(FTDI_CFLAGS) -o $@ prueba_bits.cpp $(FTDI_LIBS)

.PHONY: prueba clean
prueba: prueba_bits
	./prueba_bits

clean:
	rm -rf obj visor_enlace prueba_bits

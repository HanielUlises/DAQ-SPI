CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall -Wextra -Werror -O2

.PHONY: prueba clean

prueba: prueba_protocolo
	./prueba_protocolo

prueba_protocolo: prueba_protocolo.c daq_protocolo.h
	$(CC) $(CFLAGS) -o $@ prueba_protocolo.c

clean:
	rm -f prueba_protocolo prueba_protocolo.exe

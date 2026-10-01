#ifndef FTDI_COMPARTIDO_H
#define FTDI_COMPARTIDO_H

#include <mutex>
#include <windows.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

struct HandleFTDI {
    FT_HANDLE handle;
    bool abierto;
    HANDLE semaforo; // Candado nativo del núcleo de Windows
};

#endif
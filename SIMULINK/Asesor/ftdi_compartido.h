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

// --- DAQ-SPI: trama del firmware dsPicDaq (formato, CRC y escala del DAC) y
// elección de la interfaz del FT2232H por la que responde el dsPIC ---
#include "../../protocolo/daq_protocolo.h"
#include "../../protocolo/daq_canal.h"

struct HandleFTDI {
    FT_HANDLE handle;
    bool abierto;
    HANDLE semaforo; // <--- Candado nativo del núcleo de Windows

    // --- DAQ-SPI: lo que viaja en cada trama ---
    uint16_t valorDac;      // código del DAC que envía escribirPic
    bool salida;            // false: el dsPIC deja el DAC en 0 V
    uint8_t secuencia;      // número de la última trama enviada
    int32_t posicion;       // última posición del encoder recibida
    unsigned long tramas;
    unsigned long errores;
};

// Una transferencia SPI full-duplex de DAQ_TRAMA_LEN bytes: envía al dsPIC el
// voltaje guardado en hw y recibe la posición del encoder que el dsPIC
// preparó al terminar la trama anterior. Con el firmware dsPicDaq cada
// transferencia escribe y lee a la vez, y si pasan más de 50 ms sin tramas el
// dsPIC lleva el DAC a 0 V. Devuelve false si la respuesta no es válida.
static inline bool transferenciaPic(HandleFTDI *hw)
{
    uint8_t bufferTx[DAQ_TRAMA_LEN];
    uint8_t bufferRx[DAQ_TRAMA_LEN];
    DWORD bytesTransferidos = 0;

    WaitForSingleObject(hw->semaforo, INFINITE);

    const uint8_t anterior = hw->secuencia++;
    daq_armar_trama_pc(bufferTx, hw->secuencia,
                       hw->salida ? hw->valorDac : (uint16_t)DAQ_DAC_CERO,
                       hw->salida ? DAQ_FLAG_SALIDA_HAB : 0u);

    FT_STATUS status = SPI_ReadWrite(
        hw->handle,
        bufferRx,
        bufferTx,
        DAQ_TRAMA_LEN,
        &bytesTransferidos,
        SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
        SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
        SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE
    );

    // La respuesta trae el eco del número de la última trama que el dsPIC
    // recibió completa y con CRC correcto, que debe ser la anterior
    uint8_t eco = 0;
    uint8_t estado = 0;
    int32_t posicion = 0;
    bool valida = status == FT_OK &&
                  bytesTransferidos == DAQ_TRAMA_LEN &&
                  daq_leer_trama_pic(bufferRx, &eco, &posicion, &estado) == 0 &&
                  eco == anterior;

    hw->tramas++;
    if (valida) {
        hw->posicion = posicion;
    } else {
        hw->errores++;
    }

    ReleaseMutex(hw->semaforo);
    return valida;
}

#endif

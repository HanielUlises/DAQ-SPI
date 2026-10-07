/*
 * Canal de libMPSSE que corresponde a la interfaz A del FT2232H, la que está
 * cableada al dsPIC. Debe incluirse después de libmpsse_spi.h (o de
 * herramientas/comun/mpsse_linux.h).
 *
 * libMPSSE numera los canales en el orden en que D2XX enumera las
 * interfaces, y en Windows ese orden depende de cómo se instaló el driver:
 * en una PC el canal 0 resultó ser la interfaz B, por la que el dsPIC no
 * responde (sólo se leen 0xFF). D2XX agrega la letra de la interfaz al
 * número de serie (FT88FI80A, FT88FI80B), así que se busca el canal cuyo
 * número de serie termina en 'A'. Si ninguno lo hace (un FT232H, por
 * ejemplo), se usa el canal 0.
 */
#ifndef DAQ_CANAL_H
#define DAQ_CANAL_H

#include <string.h>

static inline DWORD daq_canal_a(DWORD canales)
{
    for (DWORD i = 0; i < canales; i++) {
        FT_DEVICE_LIST_INFO_NODE info;
        memset(&info, 0, sizeof info);
        if (SPI_GetChannelInfo(i, &info) != FT_OK) {
            continue;
        }
        const size_t n = strnlen(info.SerialNumber, sizeof info.SerialNumber);
        if (n > 0 && info.SerialNumber[n - 1] == 'A') {
            return i;
        }
    }
    return 0;
}

#endif

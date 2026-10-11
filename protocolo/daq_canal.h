/*
 * Elección del canal de libMPSSE por el que responde el dsPIC. Debe incluirse
 * después de libmpsse_spi.h (o de herramientas/comun/mpsse_linux.h) y de
 * daq_protocolo.h.
 *
 * libMPSSE numera los canales en el orden en que D2XX enumera las
 * interfaces, y en Windows ese orden depende de cómo se instaló el driver:
 * en una PC el canal 0 resultó ser la interfaz B, por la que el dsPIC no
 * responde (sólo se leen 0xFF). D2XX agrega la letra de la interfaz al
 * número de serie (FT88FI80A, FT88FI80B), y daq_canal_a busca el canal cuyo
 * número de serie termina en 'A'. Si ninguno lo hace (un FT232H, por
 * ejemplo), se usa el canal 0.
 *
 * daq_abrir_canal no depende ni de esa numeración ni de la interfaz a la que
 * se cableó el dsPIC: prueba primero el canal de la interfaz A y después los
 * demás, y se queda con el primero en que el dsPIC responde con una trama
 * válida. Las tramas de prueba llevan la salida deshabilitada (0 V) y no
 * ponen en cero el encoder.
 */
#ifndef DAQ_CANAL_H
#define DAQ_CANAL_H

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define DAQ_CANAL_RELOJ_HZ  1000000u
#define DAQ_CANAL_TRAMAS    4u      /* tramas de prueba por canal */

/* Resultado de daq_abrir_canal */
typedef struct {
    DWORD canal;            /* canal de libMPSSE abierto */
    char interfaz;          /* letra de la interfaz ('A', 'B') o '?' */
    bool responde;          /* el dsPIC respondió por ese canal */
    char detalle[160];      /* lo que se obtuvo en cada canal probado */
} DaqCanal;

/* Letra de la interfaz del canal, tomada del número de serie */
static inline char daq_canal_interfaz(DWORD canal)
{
    FT_DEVICE_LIST_INFO_NODE info;
    memset(&info, 0, sizeof info);
    if (SPI_GetChannelInfo(canal, &info) != FT_OK) {
        return '?';
    }
    const char *fin = (const char *)memchr(info.SerialNumber, '\0', sizeof info.SerialNumber);
    const size_t n = fin ? (size_t)(fin - info.SerialNumber) : sizeof info.SerialNumber;
    if (n == 0 || info.SerialNumber[n - 1] < 'A' || info.SerialNumber[n - 1] > 'D') {
        return '?';
    }
    return info.SerialNumber[n - 1];
}

static inline DWORD daq_canal_a(DWORD canales)
{
    for (DWORD i = 0; i < canales; i++) {
        if (daq_canal_interfaz(i) == 'A') {
            return i;
        }
    }
    return 0;
}

/* Abre el canal como maestro SPI en modo 0, con CS activo en bajo en xDBUS3.
 * Devuelve 0, o 1 si no se pudo abrir y 2 si falló la inicialización (en ese
 * caso lo deja cerrado). */
static inline int daq_canal_iniciar(DWORD canal, DWORD reloj, FT_HANDLE *h)
{
    if (SPI_OpenChannel(canal, h) != FT_OK) {
        return 1;
    }
    ChannelConfig conf;
    memset(&conf, 0, sizeof conf);
    conf.ClockRate = reloj;
    conf.LatencyTimer = 1;
    conf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 |
                         SPI_CONFIG_OPTION_CS_ACTIVELOW;
    if (SPI_InitChannel(*h, &conf) != FT_OK) {
        SPI_CloseChannel(*h);
        return 2;
    }
    return 0;
}

/* Envía tramas con la salida deshabilitada. La respuesta de cada una se armó
 * al terminar la anterior, así que se acepta la primera respuesta válida que
 * haga eco de la secuencia de la trama previa. Devuelve 1 si el dsPIC
 * respondió, 0 si no (en miso queda la última respuesta, para el
 * diagnóstico) y -1 si falló la transferencia. */
static inline int daq_canal_responde(FT_HANDLE h, uint8_t miso[DAQ_TRAMA_LEN])
{
    const DWORD opciones = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                           SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                           SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;
    uint8_t mosi[DAQ_TRAMA_LEN];

    memset(miso, 0, DAQ_TRAMA_LEN);
    for (unsigned k = 0; k < DAQ_CANAL_TRAMAS; k++) {
        const uint8_t seq = (uint8_t)(0xC0u + k);
        DWORD n = 0;
        daq_armar_trama_pc(mosi, seq, DAQ_DAC_CERO, 0u);
        if (SPI_ReadWrite(h, miso, mosi, DAQ_TRAMA_LEN, &n, opciones) != FT_OK ||
            n != DAQ_TRAMA_LEN) {
            return -1;
        }
        uint8_t eco, status;
        int32_t posicion;
        if (k > 0 && daq_leer_trama_pic(miso, &eco, &posicion, &status) == 0 &&
            eco == (uint8_t)(seq - 1u)) {
            return 1;
        }
    }
    return 0;
}

/* Describe una respuesta que no es del protocolo */
static inline void daq_canal_describir(const uint8_t miso[DAQ_TRAMA_LEN], char *texto,
                                       size_t n)
{
    bool unos = true, ceros = true;
    for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) {
        unos = unos && miso[i] == 0xFFu;
        ceros = ceros && miso[i] == 0x00u;
    }
    if (unos) {
        snprintf(texto, n, "solo 0xFF");
    } else if (ceros) {
        snprintf(texto, n, "solo 0x00");
    } else {
        snprintf(texto, n, "%02X %02X %02X %02X..., sin trama valida",
                 miso[0], miso[1], miso[2], miso[3]);
    }
}

/* Abre e inicializa el canal por el que responde el dsPIC. Si ninguno
 * responde, devuelve false con todo cerrado y deja en c->detalle lo que se
 * obtuvo en cada canal. */
static inline bool daq_abrir_canal(DWORD canales, DWORD reloj, FT_HANDLE *h, DaqCanal *c)
{
    const DWORD primero = daq_canal_a(canales);
    size_t usado = 0;

    memset(c, 0, sizeof *c);
    for (DWORD k = 0; k < canales; k++) {
        /* Primero el canal de la interfaz A, después los demás en orden */
        const DWORD canal = k == 0 ? primero : (k <= primero ? k - 1 : k);
        const char interfaz = daq_canal_interfaz(canal);
        uint8_t miso[DAQ_TRAMA_LEN];
        char texto[48];

        const int r = daq_canal_iniciar(canal, reloj, h);
        const int respuesta = r == 0 ? daq_canal_responde(*h, miso) : -1;
        if (respuesta == 1) {
            c->canal = canal;
            c->interfaz = interfaz;
            c->responde = true;
            return true;
        }
        if (r == 1) {
            snprintf(texto, sizeof texto, "no se pudo abrir (en uso?)");
        } else if (r == 2) {
            snprintf(texto, sizeof texto, "fallo la inicializacion");
        } else if (respuesta < 0) {
            snprintf(texto, sizeof texto, "fallo la transferencia");
        } else {
            daq_canal_describir(miso, texto, sizeof texto);
        }
        if (r == 0) {
            SPI_CloseChannel(*h);
        }
        *h = NULL;
        if (usado < sizeof c->detalle) {
            int escrito;
            if (interfaz == '?') {
                escrito = snprintf(c->detalle + usado, sizeof c->detalle - usado, "%scanal %u: %s",
                                   usado ? "; " : "", (unsigned)canal, texto);
            } else {
                escrito = snprintf(c->detalle + usado, sizeof c->detalle - usado,
                                   "%sinterfaz %c: %s", usado ? "; " : "", interfaz, texto);
            }
            if (escrito > 0) {
                usado += (size_t)escrito;
            }
        }
    }
    return false;
}

#endif

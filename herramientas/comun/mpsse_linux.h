/*
 * Subconjunto de la API de libMPSSE-SPI implementado sobre libftdi1, para
 * compilar prueba_enlace en Linux. Reproduce sólo lo que usa el programa:
 * canal A del FT2232H como maestro SPI en modo 0, CS activo en bajo en
 * ADBUS3 y transferencias full-duplex enmarcadas por CS.
 *
 * El FT2232H de la tarjeta tiene la EEPROM reprogramada con VID 0x2099 y
 * PID 0x0001 ("Custom DAC USB Bridge", el mismo identificador que reconoce
 * el driver modificado de DRIVER FTDI/). Se buscan ese identificador y el de
 * fábrica. 70-daq-spi.rules da acceso al dispositivo sin sudo.
 *
 * libftdi1 desliga el módulo ftdi_sio del canal A al abrirlo, por lo que no
 * hace falta descargarlo.
 */
#ifndef MPSSE_LINUX_H
#define MPSSE_LINUX_H

#include <ftdi.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

typedef uint32_t DWORD;
typedef uint32_t FT_STATUS;
typedef struct ftdi_context *FT_HANDLE;

enum {
    FT_OK = 0,
    FT_INVALID_HANDLE = 1,
    FT_DEVICE_NOT_FOUND = 2,
    FT_DEVICE_NOT_OPENED = 3,
    FT_IO_ERROR = 4,
    FT_OTHER_ERROR = 18
};

#define SPI_CONFIG_OPTION_MODE0                 0x00000000
#define SPI_CONFIG_OPTION_CS_DBUS3              0x00000000
#define SPI_CONFIG_OPTION_CS_ACTIVELOW          0x00000020

#define SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES      0x00000000
#define SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE  0x00000002
#define SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE 0x00000004

typedef struct ChannelConfig_t {
    DWORD ClockRate;
    uint8_t LatencyTimer;
    DWORD configOptions;
    DWORD Pin;
    uint16_t currentPinState;
} ChannelConfig;

/* Comandos MPSSE (AN_108 de FTDI) */
enum {
    MPSSE_ESCRIBIR_LEER_MSB = 0x31, /* bytes, salida en flanco -, entrada en + */
    MPSSE_PINES_BAJOS = 0x80,
    MPSSE_LAZO_ON = 0x84,
    MPSSE_LAZO_OFF = 0x85,
    MPSSE_DIVISOR = 0x86,
    MPSSE_ENVIAR_YA = 0x87,
    MPSSE_SIN_DIV5 = 0x8A,
    MPSSE_SIN_3FASES = 0x8D,
    MPSSE_SIN_ADAPTATIVO = 0x97,
    MPSSE_CMD_INVALIDO = 0xAA
};

/* ADBUS0 SCK, ADBUS1 MOSI y ADBUS3 CS son salidas; ADBUS2 MISO es entrada */
static const uint8_t kMpsseDireccion = 0x0B;
static const uint8_t kMpsseCsAlto = 0x08;
static const uint8_t kMpsseCsBajo = 0x00;

static int mpsse_escribir(FT_HANDLE h, const uint8_t *datos, int n)
{
    return ftdi_write_data(h, datos, n) == n ? 0 : -1;
}

/* Lee exactamente n bytes, con un límite de 1 s */
static int mpsse_leer(FT_HANDLE h, uint8_t *datos, int n)
{
    struct timespec t0, t;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    int leidos = 0;
    while (leidos < n) {
        int r = ftdi_read_data(h, datos + leidos, n - leidos);
        if (r < 0) {
            return -1;
        }
        leidos += r;
        clock_gettime(CLOCK_MONOTONIC, &t);
        if (r == 0 && (t.tv_sec - t0.tv_sec) * 1000000000L + (t.tv_nsec - t0.tv_nsec) > 1000000000L) {
            return -1;
        }
    }
    return 0;
}

static inline FT_STATUS Init_libMPSSE(void) { return FT_OK; }
static inline FT_STATUS Cleanup_libMPSSE(void) { return FT_OK; }

/* Identificadores USB aceptados: el de la tarjeta y el de fábrica */
static const struct { int vid, pid; } kMpsseIds[] = {
    { 0x2099, 0x0001 },
    { 0x0403, 0x6010 },
};
static const int kMpsseNumIds = (int)(sizeof kMpsseIds / sizeof kMpsseIds[0]);

static FT_STATUS SPI_GetNumChannels(DWORD *canales)
{
    *canales = 0;
    for (int i = 0; i < kMpsseNumIds; i++) {
        struct ftdi_context *ctx = ftdi_new();
        struct ftdi_device_list *lista = NULL;
        if (ctx == NULL) {
            return FT_OTHER_ERROR;
        }
        int n = ftdi_usb_find_all(ctx, &lista, kMpsseIds[i].vid, kMpsseIds[i].pid);
        ftdi_list_free(&lista);
        ftdi_free(ctx);
        if (n < 0) {
            return FT_OTHER_ERROR;
        }
        *canales += (DWORD)n;
    }
    return FT_OK;
}

typedef struct {
    DWORD Flags;
    DWORD Type;
    DWORD ID;
    DWORD LocId;
    char SerialNumber[16];
    char Description[64];
    FT_HANDLE ftHandle;
} FT_DEVICE_LIST_INFO_NODE;

/* Aquí cada canal es la interfaz A de un dispositivo, así que el número de
 * serie termina en 'A' como lo reporta D2XX en Windows */
static FT_STATUS SPI_GetChannelInfo(DWORD indice, FT_DEVICE_LIST_INFO_NODE *info)
{
    memset(info, 0, sizeof *info);
    snprintf(info->SerialNumber, sizeof info->SerialNumber, "%uA", (unsigned)indice);
    snprintf(info->Description, sizeof info->Description, "Canal A");
    return FT_OK;
}

/* Abre el canal A del dispositivo número indice, contando todos los
 * identificadores aceptados en el orden de kMpsseIds */
static FT_STATUS SPI_OpenChannel(DWORD indice, FT_HANDLE *h)
{
    for (int i = 0; i < kMpsseNumIds; i++) {
        struct ftdi_context *ctx = ftdi_new();
        struct ftdi_device_list *lista = NULL;
        if (ctx == NULL) {
            return FT_OTHER_ERROR;
        }
        int n = ftdi_usb_find_all(ctx, &lista, kMpsseIds[i].vid, kMpsseIds[i].pid);
        if (n > 0 && indice < (DWORD)n) {
            struct ftdi_device_list *d = lista;
            for (DWORD k = 0; k < indice; k++) {
                d = d->next;
            }
            ftdi_set_interface(ctx, INTERFACE_A);
            ctx->module_detach_mode = AUTO_DETACH_REATACH_SIO_MODULE;
            int r = ftdi_usb_open_dev(ctx, d->dev);
            ftdi_list_free(&lista);
            if (r < 0) {
                fprintf(stderr, "libftdi: %s\n", ftdi_get_error_string(ctx));
                if (r == -4) {
                    fprintf(stderr, "Probablemente sin permiso sobre el dispositivo USB; "
                                    "instalar herramientas/prueba_enlace/70-daq-spi.rules.\n");
                }
                ftdi_free(ctx);
                return FT_DEVICE_NOT_OPENED;
            }
            *h = ctx;
            return FT_OK;
        }
        ftdi_list_free(&lista);
        ftdi_free(ctx);
        if (n > 0) {
            indice -= (DWORD)n;
        }
    }
    return FT_DEVICE_NOT_FOUND;
}

static FT_STATUS SPI_InitChannel(FT_HANDLE h, const ChannelConfig *conf)
{
    if (ftdi_usb_reset(h) < 0 ||
        ftdi_set_latency_timer(h, conf->LatencyTimer) < 0 ||
        ftdi_set_bitmode(h, 0, BITMODE_RESET) < 0 ||
        ftdi_set_bitmode(h, 0, BITMODE_MPSSE) < 0 ||
        ftdi_tcioflush(h) < 0) {
        fprintf(stderr, "libftdi: %s\n", ftdi_get_error_string(h));
        return FT_IO_ERROR;
    }

    /* Sincronización: el MPSSE responde 0xFA seguido del comando inválido */
    const uint8_t invalido = MPSSE_CMD_INVALIDO;
    uint8_t eco[2];
    if (mpsse_escribir(h, &invalido, 1) || mpsse_leer(h, eco, 2) ||
        eco[0] != 0xFA || eco[1] != MPSSE_CMD_INVALIDO) {
        fprintf(stderr, "El MPSSE no respondió a la sincronización.\n");
        return FT_IO_ERROR;
    }

    /* Reloj base de 60 MHz: SCK = 60 MHz / (2 (1 + divisor)) */
    const uint16_t divisor = (uint16_t)(30000000u / conf->ClockRate - 1u);
    const uint8_t cmd[] = {
        MPSSE_SIN_DIV5, MPSSE_SIN_ADAPTATIVO, MPSSE_SIN_3FASES,
        MPSSE_DIVISOR, (uint8_t)divisor, (uint8_t)(divisor >> 8),
        MPSSE_LAZO_OFF,
        MPSSE_PINES_BAJOS, kMpsseCsAlto, kMpsseDireccion
    };
    if (mpsse_escribir(h, cmd, (int)sizeof cmd)) {
        return FT_IO_ERROR;
    }
    return FT_OK;
}

/* Une internamente MOSI con MISO dentro del FT2232H (no existe en libMPSSE) */
static FT_STATUS SPI_Lazo(FT_HANDLE h, bool activo)
{
    const uint8_t cmd = activo ? MPSSE_LAZO_ON : MPSSE_LAZO_OFF;
    return mpsse_escribir(h, &cmd, 1) ? FT_IO_ERROR : FT_OK;
}

static FT_STATUS SPI_ReadWrite(FT_HANDLE h, uint8_t *rx, const uint8_t *tx, DWORD n,
                               DWORD *transferidos, DWORD opciones)
{
    uint8_t cmd[3 + 3 + 64 + 3 + 1];
    int k = 0;

    *transferidos = 0;
    if (n == 0 || n > 64) {
        return FT_OTHER_ERROR;
    }
    if (opciones & SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE) {
        cmd[k++] = MPSSE_PINES_BAJOS;
        cmd[k++] = kMpsseCsBajo;
        cmd[k++] = kMpsseDireccion;
    }
    cmd[k++] = MPSSE_ESCRIBIR_LEER_MSB;
    cmd[k++] = (uint8_t)(n - 1u);
    cmd[k++] = (uint8_t)((n - 1u) >> 8);
    memcpy(&cmd[k], tx, n);
    k += (int)n;
    if (opciones & SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE) {
        cmd[k++] = MPSSE_PINES_BAJOS;
        cmd[k++] = kMpsseCsAlto;
        cmd[k++] = kMpsseDireccion;
    }
    cmd[k++] = MPSSE_ENVIAR_YA;

    if (mpsse_escribir(h, cmd, k) || mpsse_leer(h, rx, (int)n)) {
        return FT_IO_ERROR;
    }
    *transferidos = n;
    return FT_OK;
}

static FT_STATUS SPI_CloseChannel(FT_HANDLE h)
{
    ftdi_set_bitmode(h, 0, BITMODE_RESET);
    ftdi_usb_close(h);
    ftdi_free(h);
    return FT_OK;
}

#endif

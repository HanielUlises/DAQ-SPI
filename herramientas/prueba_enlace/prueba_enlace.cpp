/*
 * Prueba del enlace PC <-> dsPIC sin Simulink (etapa 2 de docs/propuesta.pdf).
 *
 * Envía N tramas con SPI_ReadWrite y contabiliza errores de encabezado, CRC,
 * número de secuencia y los reportados por el dsPIC en el byte status. Mide
 * además la duración de cada transferencia. El criterio de aceptación es la
 * ausencia de errores en al menos 100 000 tramas.
 *
 * Por omisión la salida analógica queda deshabilitada (0 V) aunque el campo
 * del DAC varía, de modo que se ejercita el protocolo sin mover el motor. Con
 * -salida se envía una rampa triangular de +/-1 V para verificarla con
 * osciloscopio.
 *
 * Compilación en Windows (x64 Native Tools Command Prompt de Visual Studio):
 *   compilar.bat
 * libmpsse.dll y msvcr120.dll deben estar junto al ejecutable. Se incluye un
 * ejecutable ya compilado.
 *
 * Compilación en Linux (requiere libftdi1, ver mpsse_linux.h):
 *   make
 * En Linux, -lazo une MOSI con MISO dentro del FT2232H y verifica que cada
 * byte recibido sea igual al enviado. Sirve para probar el FT2232H y medir la
 * duración de las transferencias sin el dsPIC conectado.
 *
 * Uso: prueba_enlace [-n tramas] [-periodo us] [-salida] [-lazo]
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#ifdef _WIN32
#include <windows.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"
#else
#include <csignal>
#include "mpsse_linux.h"
#endif
#include "../../protocolo/daq_protocolo.h"

static const DWORD kReloj = 1000000;
static const DWORD kOpciones = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

#ifdef _WIN32
static volatile LONG detener = 0;

static BOOL WINAPI manejar_ctrl_c(DWORD evento)
{
    if (evento == CTRL_C_EVENT || evento == CTRL_BREAK_EVENT) {
        InterlockedExchange(&detener, 1);
        return TRUE;
    }
    return FALSE;
}

/* Tiempo en microsegundos desde un origen arbitrario */
static double reloj_us(void)
{
    static double us_por_cuenta = 0.0;
    LARGE_INTEGER t;
    if (us_por_cuenta == 0.0) {
        LARGE_INTEGER frec;
        QueryPerformanceFrequency(&frec);
        us_por_cuenta = 1e6 / (double)frec.QuadPart;
    }
    QueryPerformanceCounter(&t);
    return (double)t.QuadPart * us_por_cuenta;
}
#else
static volatile sig_atomic_t detener = 0;

static void manejar_ctrl_c(int)
{
    detener = 1;
}

static double reloj_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec * 1e6 + (double)t.tv_nsec * 1e-3;
}
#endif

struct Contadores {
    unsigned long tramas;
    unsigned long err_usb;
    unsigned long err_eco;
    unsigned long err_inicio;
    unsigned long err_crc;
    unsigned long err_seq;
    unsigned long pic_crc;
    unsigned long pic_inicio;
    unsigned long pic_longitud;
    unsigned long pic_vigilancia;
};

/* Histograma de la duración de cada transferencia, en intervalos de 250 us */
static const int kCubetas = 16;
static const double kAnchoCubeta = 250.0;

static uint16_t rampa(unsigned long k)
{
    /* Triangular de +/-1 V con periodo de 2000 tramas */
    const double amplitud = 1.0;
    double fase = (double)(k % 2000u) / 2000.0;
    double v = (fase < 0.5) ? (-amplitud + 4.0 * amplitud * fase)
                            : (3.0 * amplitud - 4.0 * amplitud * fase);
    return daq_voltaje_a_dac(v);
}

/* Generador pseudoaleatorio (xorshift) para variar el campo del DAC */
static uint16_t aleatorio(void)
{
    static uint32_t x = 2463534242u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return (uint16_t)x;
}

static void enviar_reposo(FT_HANDLE h, uint8_t seq)
{
    uint8_t tx[DAQ_TRAMA_LEN];
    uint8_t rx[DAQ_TRAMA_LEN];
    DWORD n = 0;

    daq_armar_trama_pc(tx, seq, DAQ_DAC_CERO, 0u);
    SPI_ReadWrite(h, rx, tx, DAQ_TRAMA_LEN, &n, kOpciones);
}

int main(int argc, char **argv)
{
    unsigned long total = 100000ul;
    double periodo_us = 0.0;
    bool salida = false;
    bool lazo = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-n") && i + 1 < argc) {
            total = strtoul(argv[++i], NULL, 10);
        } else if (!strcmp(argv[i], "-periodo") && i + 1 < argc) {
            periodo_us = atof(argv[++i]);
        } else if (!strcmp(argv[i], "-salida")) {
            salida = true;
#ifndef _WIN32
        } else if (!strcmp(argv[i], "-lazo")) {
            lazo = true;
#endif
        } else {
            fprintf(stderr, "Uso: %s [-n tramas] [-periodo us] [-salida]%s\n", argv[0],
#ifdef _WIN32
                    "");
#else
                    " [-lazo]");
#endif
            return 2;
        }
    }

#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCtrlHandler(manejar_ctrl_c, TRUE);
#else
    signal(SIGINT, manejar_ctrl_c);
#endif
    Init_libMPSSE();

    DWORD canales = 0;
    FT_HANDLE h = NULL;
    if (SPI_GetNumChannels(&canales) != FT_OK || canales == 0) {
        fprintf(stderr, "No se detectó ningún FT2232H.\n");
        Cleanup_libMPSSE();
        return 2;
    }
    if (SPI_OpenChannel(0, &h) != FT_OK) {
        fprintf(stderr, "No se pudo abrir el canal 0 (¿está en uso?).\n");
        Cleanup_libMPSSE();
        return 2;
    }

    ChannelConfig conf;
    memset(&conf, 0, sizeof conf);
    conf.ClockRate = kReloj;
    conf.LatencyTimer = 1;
    conf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 |
                         SPI_CONFIG_OPTION_CS_ACTIVELOW;
    if (SPI_InitChannel(h, &conf) != FT_OK) {
        fprintf(stderr, "Falló la inicialización del canal SPI.\n");
        SPI_CloseChannel(h);
        Cleanup_libMPSSE();
        return 2;
    }

#ifndef _WIN32
    if (lazo && SPI_Lazo(h, true) != FT_OK) {
        fprintf(stderr, "No se pudo activar el lazo interno.\n");
        SPI_CloseChannel(h);
        return 2;
    }
#endif

    printf("Tramas: %lu, periodo: %s, salida: %s%s\n", total,
           periodo_us > 0.0 ? "fijo" : "lo más rápido posible",
           salida ? "rampa +/-1 V" : "deshabilitada (0 V)",
           lazo ? ", lazo interno MOSI -> MISO (sin dsPIC)" : "");

    Contadores c;
    memset(&c, 0, sizeof c);
    unsigned long histograma[kCubetas];
    memset(histograma, 0, sizeof histograma);
    double t_min = 1e30, t_max = 0.0, t_suma = 0.0;

    uint8_t tx[DAQ_TRAMA_LEN];
    uint8_t rx[DAQ_TRAMA_LEN];
    uint8_t seq_anterior = 0;

    const double inicio = reloj_us();
    double siguiente = inicio;

    for (unsigned long k = 0; k < total && !detener; k++) {
        const uint8_t seq = (uint8_t)k;
        const uint16_t dac = salida ? rampa(k) : aleatorio();
        const uint8_t flags = salida ? DAQ_FLAG_SALIDA_HAB : 0u;
        DWORD n = 0;

        daq_armar_trama_pc(tx, seq, dac, flags);

        if (periodo_us > 0.0) {
            siguiente += periodo_us;
            while (reloj_us() < siguiente) {
            }
        }

        const double t0 = reloj_us();
        FT_STATUS st = SPI_ReadWrite(h, rx, tx, DAQ_TRAMA_LEN, &n, kOpciones);
        const double dt = reloj_us() - t0;
        t_suma += dt;
        if (dt < t_min) t_min = dt;
        if (dt > t_max) t_max = dt;
        int cubeta = (int)(dt / kAnchoCubeta);
        histograma[cubeta < kCubetas ? cubeta : kCubetas - 1]++;
        c.tramas++;

        if (st != FT_OK || n != DAQ_TRAMA_LEN) {
            c.err_usb++;
            continue;
        }

        if (lazo) {
            if (memcmp(rx, tx, DAQ_TRAMA_LEN) != 0) c.err_eco++;
            if ((k + 1) % 10000u == 0) printf("  %7lu tramas\n", k + 1);
            continue;
        }

        uint8_t seq_eco, status;
        int32_t posicion = 0;
        int r = daq_leer_trama_pic(rx, &seq_eco, &posicion, &status);
        if (r == -1) {
            c.err_inicio++;
        } else if (r == -2) {
            c.err_crc++;
        } else if (k > 0) {
            /* La primera respuesta refleja el estado previo a la prueba */
            if (seq_eco != seq_anterior) c.err_seq++;
            if (status & DAQ_STATUS_ERR_CRC) c.pic_crc++;
            if (status & DAQ_STATUS_ERR_INICIO) c.pic_inicio++;
            if (status & DAQ_STATUS_ERR_LONGITUD) c.pic_longitud++;
            if (status & DAQ_STATUS_VIGILANCIA) c.pic_vigilancia++;
        }
        seq_anterior = seq;

        if ((k + 1) % 10000u == 0) {
            printf("  %7lu tramas, posición %ld\n", k + 1, (long)posicion);
        }
    }

    const double fin = reloj_us();
#ifndef _WIN32
    if (lazo) SPI_Lazo(h, false);
#endif
    enviar_reposo(h, (uint8_t)c.tramas);
    SPI_CloseChannel(h);
    Cleanup_libMPSSE();

    const double t_total = (fin - inicio) * 1e-6;
    const unsigned long errores = c.err_usb + c.err_eco + c.err_inicio + c.err_crc + c.err_seq +
                                  c.pic_crc + c.pic_inicio + c.pic_longitud +
                                  c.pic_vigilancia;

    printf("\nTramas enviadas:            %lu en %.2f s (%.0f tramas/s)\n", c.tramas,
           t_total, t_total > 0.0 ? (double)c.tramas / t_total : 0.0);
    printf("Errores en la PC\n");
    printf("  USB / longitud:           %lu\n", c.err_usb);
    if (lazo) {
        printf("  eco distinto del envío:   %lu\n", c.err_eco);
    }
    printf("  encabezado:               %lu\n", c.err_inicio);
    printf("  CRC:                      %lu\n", c.err_crc);
    printf("  número de secuencia:      %lu\n", c.err_seq);
    printf("Errores reportados por el dsPIC\n");
    printf("  CRC:                      %lu\n", c.pic_crc);
    printf("  encabezado:               %lu\n", c.pic_inicio);
    printf("  longitud:                 %lu\n", c.pic_longitud);
    printf("  vigilancia:               %lu\n", c.pic_vigilancia);
    if (c.tramas > 0) {
        printf("Duración de SPI_ReadWrite:  mín %.0f us, media %.0f us, máx %.0f us\n",
               t_min, t_suma / (double)c.tramas, t_max);
        for (int i = 0; i < kCubetas; i++) {
            if (histograma[i] == 0) continue;
            if (i < kCubetas - 1) {
                printf("  %5.0f - %5.0f us: %lu\n", i * kAnchoCubeta,
                       (i + 1) * kAnchoCubeta, histograma[i]);
            } else {
                printf("  >= %5.0f us:       %lu\n", i * kAnchoCubeta, histograma[i]);
            }
        }
    }
    printf("\nResultado: %s\n", errores == 0 ? "SIN ERRORES" : "CON ERRORES");
    return errores == 0 ? 0 : 1;
}

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
 * Compilación en Linux (requiere libftdi1, ver ../comun/mpsse_linux.h):
 *   make
 * En Linux, -lazo une MOSI con MISO dentro del FT2232H y verifica que cada
 * byte recibido sea igual al enviado. Sirve para probar el FT2232H y medir la
 * duración de las transferencias sin el dsPIC conectado.
 *
 * Con -registro se escribe un CSV con una línea por trama (tiempos, bytes
 * enviados y recibidos, resultado), del que herramientas/reporte genera el
 * reporte de la prueba.
 *
 * -reloj cambia la frecuencia de SCK (por omisión 1 MHz, la de daqPic).
 *
 * Uso: prueba_enlace [-n tramas] [-periodo us] [-reloj Hz] [-salida] [-lazo]
 *                    [-registro archivo.csv]
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#ifndef _WIN32
#include <csignal>
#endif
#include "../comun/enlace_prueba.h"

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
#else
static volatile sig_atomic_t detener = 0;

static void manejar_ctrl_c(int)
{
    detener = 1;
}
#endif

int main(int argc, char **argv)
{
    unsigned long total = 100000ul;
    double periodo_us = 0.0;
    DWORD reloj = 1000000;
    bool salida = false;
    bool lazo = false;
    const char *ruta_registro = NULL;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-n") && i + 1 < argc) {
            total = strtoul(argv[++i], NULL, 10);
        } else if (!strcmp(argv[i], "-periodo") && i + 1 < argc) {
            periodo_us = atof(argv[++i]);
        } else if (!strcmp(argv[i], "-reloj") && i + 1 < argc) {
            reloj = (DWORD)strtoul(argv[++i], NULL, 10);
        } else if (!strcmp(argv[i], "-salida")) {
            salida = true;
        } else if (!strcmp(argv[i], "-registro") && i + 1 < argc) {
            ruta_registro = argv[++i];
#ifndef _WIN32
        } else if (!strcmp(argv[i], "-lazo")) {
            lazo = true;
#endif
        } else {
            fprintf(stderr, "Uso: %s [-n tramas] [-periodo us] [-reloj Hz] [-salida]%s"
                            " [-registro archivo.csv]\n", argv[0],
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
    FILE *registro = NULL;
    if (ruta_registro != NULL) {
        registro = fopen(ruta_registro, "w");
        if (registro == NULL) {
            fprintf(stderr, "No se pudo crear %s.\n", ruta_registro);
            return 2;
        }
        char comando[512] = "";
        for (int i = 0; i < argc; i++) {
            if (i > 0) strncat(comando, " ", sizeof comando - strlen(comando) - 1);
            strncat(comando, argv[i], sizeof comando - strlen(comando) - 1);
        }
        registro_encabezado(registro, comando);
    }

    FT_HANDLE h = NULL;
    char error[128];
    if (!enlace_abrir(reloj, lazo, &h, error, sizeof error)) {
        fprintf(stderr, "%s\n", error);
        return 2;
    }

    printf("Tramas: %lu, SCK: %lu kHz, periodo: %s, salida: %s%s\n", total,
           (unsigned long)(reloj / 1000u),
           periodo_us > 0.0 ? "fijo" : "lo más rápido posible",
           salida ? "rampa +/-1 V" : "deshabilitada (0 V)",
           lazo ? ", lazo interno MOSI -> MISO (sin dsPIC)" : "");

    EstadoPrueba e;
    prueba_iniciar(&e);
    const Contadores &c = e.c;

    const double inicio = reloj_us();
    double siguiente = inicio;

    for (unsigned long k = 0; k < total && !detener; k++) {
        Trama t;
        const uint16_t dac = salida ? rampa(k) : aleatorio();
        const uint8_t flags = salida ? DAQ_FLAG_SALIDA_HAB : 0u;
        DWORD n = 0;

        t.k = k;
        daq_armar_trama_pc(t.tx, (uint8_t)k, dac, flags);

        if (periodo_us > 0.0) {
            siguiente += periodo_us;
            while (reloj_us() < siguiente) {
            }
        }

        const double t0 = reloj_us();
        t.t_us = t0 - inicio;
        FT_STATUS st = SPI_ReadWrite(h, t.rx, t.tx, DAQ_TRAMA_LEN, &n, kOpciones);
        t.dt_us = reloj_us() - t0;
        prueba_procesar(&e, &t, st, n, lazo);
        registrar(registro, &t);

        if (t.resultado != RES_USB && (k + 1) % 10000u == 0) {
            if (lazo) {
                printf("  %7lu tramas\n", k + 1);
            } else {
                printf("  %7lu tramas, posición %ld\n", k + 1, (long)t.posicion);
            }
        }
    }

    const double fin = reloj_us();
    enlace_cerrar(h, lazo, (uint8_t)c.tramas);
    if (registro != NULL) fclose(registro);

    const double t_total = (fin - inicio) * 1e-6;
    const unsigned long reinicios = prueba_reinicios(&c);
    const unsigned long errores = prueba_errores(&c);

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
    printf("  reinicios:                %lu\n", reinicios);
    for (int i = 1; i < 16; i++) {
        if (c.pic_reinicio[i] > 0) {
            printf("    %-30s %lu\n", kCausasReinicio[i], c.pic_reinicio[i]);
        }
    }
    if (c.tramas > 0) {
        printf("Duración de SPI_ReadWrite:  mín %.0f us, media %.0f us, máx %.0f us\n",
               e.t_min, e.t_suma / (double)c.tramas, e.t_max);
        for (int i = 0; i < kCubetas; i++) {
            if (e.histograma[i] == 0) continue;
            if (i < kCubetas - 1) {
                printf("  %5.0f - %5.0f us: %lu\n", i * kAnchoCubeta,
                       (i + 1) * kAnchoCubeta, e.histograma[i]);
            } else {
                printf("  >= %5.0f us:       %lu\n", i * kAnchoCubeta, e.histograma[i]);
            }
        }
    }
    printf("\nResultado: %s\n", errores == 0 ? "SIN ERRORES" : "CON ERRORES");
    return errores == 0 ? 0 : 1;
}

/*
 * Núcleo de la prueba del enlace PC <-> dsPIC, compartido por prueba_enlace
 * (consola) y visor_enlace (interfaz gráfica): apertura del canal SPI del
 * FT2232H, campo del DAC de cada trama, clasificación de la respuesta,
 * contadores y registro CSV.
 *
 * En Windows usa libMPSSE-SPI (ftd2xx.h y libmpsse_spi.h); en Linux, el
 * subconjunto implementado sobre libftdi1 en mpsse_linux.h.
 */
#ifndef ENLACE_PRUEBA_H
#define ENLACE_PRUEBA_H

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <ctime>

#ifdef _WIN32
#include <windows.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"
#else
#include "mpsse_linux.h"
#endif
#include "../../protocolo/daq_protocolo.h"
#include "../../protocolo/daq_canal.h"

static const DWORD kOpciones = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

/* Tiempo en microsegundos desde un origen arbitrario */
#ifdef _WIN32
static inline double reloj_us(void)
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
static inline double reloj_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec * 1e6 + (double)t.tv_nsec * 1e-3;
}
#endif

/* Resultado de una trama, en el orden y con los nombres del registro CSV */
enum Resultado {
    RES_OK, RES_PRIMERA, RES_USB, RES_ECO, RES_INICIO, RES_CRC, RES_SEQ, RES_NUM
};

static const char *const kResultados[RES_NUM] = {
    "ok", "primera", "usb", "eco", "inicio", "crc", "seq",
};

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
    unsigned long pic_reinicio[16];     /* por causa (DAQ_REINICIO_*) */
};

static const char *const kCausasReinicio[16] = {
    "", "encendido", "bajo voltaje", "MCLR", "watchdog", "reset por software",
    "opcode ilegal / W sin inicializar", "configuración", "trampa: oscilador",
    "trampa: pila", "trampa: dirección", "trampa: matemática", "trampa: DMA",
    "trampa: hard", "trampa: otra", "desconocida",
};

/* Histograma de la duración de cada transferencia, en intervalos de 250 us */
static const int kCubetas = 16;
static const double kAnchoCubeta = 250.0;

struct EstadoPrueba {
    Contadores c;
    unsigned long histograma[kCubetas];
    double t_min, t_max, t_suma;
    uint8_t seq_anterior;
};

/* Una trama de la prueba: lo enviado, lo recibido y su clasificación */
struct Trama {
    unsigned long k;
    double t_us;                /* inicio de la transferencia desde el inicio de la prueba */
    double dt_us;               /* duración de SPI_ReadWrite */
    uint8_t tx[DAQ_TRAMA_LEN];
    uint8_t rx[DAQ_TRAMA_LEN];
    Resultado resultado;
    int status;                 /* -1 si la respuesta no es válida o en lazo */
    int32_t posicion;           /* válida si status >= 0 */
};

static inline void prueba_iniciar(EstadoPrueba *e)
{
    memset(e, 0, sizeof *e);
    e->t_min = 1e30;
}

static inline unsigned long prueba_reinicios(const Contadores *c)
{
    unsigned long n = 0;
    for (int i = 1; i < 16; i++) n += c->pic_reinicio[i];
    return n;
}

static inline unsigned long prueba_errores(const Contadores *c)
{
    return c->err_usb + c->err_eco + c->err_inicio + c->err_crc + c->err_seq +
           c->pic_crc + c->pic_inicio + c->pic_longitud + c->pic_vigilancia +
           prueba_reinicios(c);
}

static inline uint16_t rampa(unsigned long k)
{
    /* Triangular de +/-1 V con periodo de 2000 tramas */
    const double amplitud = 1.0;
    double fase = (double)(k % 2000u) / 2000.0;
    double v = (fase < 0.5) ? (-amplitud + 4.0 * amplitud * fase)
                            : (3.0 * amplitud - 4.0 * amplitud * fase);
    return daq_voltaje_a_dac(v);
}

/* Generador pseudoaleatorio (xorshift) para variar el campo del DAC */
static inline uint16_t aleatorio(void)
{
    static uint32_t x = 2463534242u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return (uint16_t)x;
}

/* Banderas del status de la trama t que cuentan como error. La respuesta a la
 * segunda trama (k = 1) trae lo que pasó antes de la prueba: la vigilancia
 * expira siempre que el enlace estuvo inactivo más de 50 ms, por lo que esa
 * bandera no se cuenta. El registro conserva el status completo. */
static inline uint8_t status_contable(const Trama *t)
{
    uint8_t s = (uint8_t)t->status;
    if (t->k == 1) s &= (uint8_t)~DAQ_STATUS_VIGILANCIA;
    return s;
}

/* Clasifica la respuesta de la trama t (ya con tx, rx, k, t_us y dt_us) y
 * actualiza los contadores. st y n son lo que devolvió SPI_ReadWrite. */
static inline void prueba_procesar(EstadoPrueba *e, Trama *t, FT_STATUS st, DWORD n, bool lazo)
{
    Contadores &c = e->c;
    const uint8_t seq = t->tx[DAQ_PC_SEQ];

    e->t_suma += t->dt_us;
    if (t->dt_us < e->t_min) e->t_min = t->dt_us;
    if (t->dt_us > e->t_max) e->t_max = t->dt_us;
    int cubeta = (int)(t->dt_us / kAnchoCubeta);
    e->histograma[cubeta < kCubetas ? cubeta : kCubetas - 1]++;
    c.tramas++;
    t->status = -1;
    t->posicion = 0;

    if (st != FT_OK || n != DAQ_TRAMA_LEN) {
        c.err_usb++;
        t->resultado = RES_USB;
        return;
    }

    if (lazo) {
        const bool igual = memcmp(t->rx, t->tx, DAQ_TRAMA_LEN) == 0;
        if (!igual) c.err_eco++;
        t->resultado = igual ? RES_OK : RES_ECO;
        return;
    }

    uint8_t seq_eco, status;
    int r = daq_leer_trama_pic(t->rx, &seq_eco, &t->posicion, &status);
    if (r == -1) {
        c.err_inicio++;
        t->resultado = RES_INICIO;
    } else if (r == -2) {
        c.err_crc++;
        t->resultado = RES_CRC;
    } else if (t->k == 0) {
        /* La primera respuesta refleja el estado previo a la prueba */
        t->resultado = RES_PRIMERA;
        t->status = status;
    } else {
        if (seq_eco != e->seq_anterior) c.err_seq++;
        t->resultado = seq_eco != e->seq_anterior ? RES_SEQ : RES_OK;
        t->status = status;
        status = status_contable(t);
        if (status & DAQ_STATUS_ERR_CRC) c.pic_crc++;
        if (status & DAQ_STATUS_ERR_INICIO) c.pic_inicio++;
        if (status & DAQ_STATUS_ERR_LONGITUD) c.pic_longitud++;
        if (status & DAQ_STATUS_VIGILANCIA) c.pic_vigilancia++;
        c.pic_reinicio[(status & DAQ_STATUS_REINICIO) >> DAQ_STATUS_REINICIO_POS]++;
    }
    e->seq_anterior = seq;
}

/* Encabezado del registro CSV. comando son los argumentos (equivalentes) de
 * prueba_enlace, de los que herramientas/reporte toma la configuración; visor,
 * si no es NULL, describe la señal que generó visor_enlace. */
static inline void registro_encabezado(FILE *f, const char *comando, const char *visor = NULL)
{
    char fecha[32];
    time_t ahora = time(NULL);
    strftime(fecha, sizeof fecha, "%Y-%m-%d %H:%M:%S", localtime(&ahora));
    fprintf(f, "# comando: %s\n# fecha: %s\n# plataforma: %s\n", comando, fecha,
#ifdef _WIN32
            "Windows");
#else
            "Linux");
#endif
    if (visor != NULL) fprintf(f, "# visor: %s\n", visor);
    fprintf(f, "trama,t_us,dt_us,mosi,miso,resultado,status\n");
}

static inline void escribir_bytes(FILE *f, const uint8_t *b)
{
    for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) {
        fprintf(f, "%02X", b[i]);
    }
}

/* Una línea del registro por trama */
static inline void registrar(FILE *f, const Trama *t)
{
    if (f == NULL) return;
    fprintf(f, "%lu,%.1f,%.1f,", t->k, t->t_us, t->dt_us);
    escribir_bytes(f, t->tx);
    fputc(',', f);
    escribir_bytes(f, t->rx);
    fprintf(f, ",%s,", kResultados[t->resultado]);
    if (t->status >= 0) fprintf(f, "%02X", t->status);
    fputc('\n', f);
}

/* Abre la interfaz A (daq_canal.h) como maestro SPI en modo 0 con SCK = reloj. Con lazo (sólo
 * en Linux) une MOSI con MISO dentro del FT2232H. Si falla, deja el motivo en
 * error y devuelve false. */
static inline bool enlace_abrir(DWORD reloj, bool lazo, FT_HANDLE *h, char *error, size_t n)
{
    Init_libMPSSE();

    DWORD canales = 0;
    *h = NULL;
    if (SPI_GetNumChannels(&canales) != FT_OK || canales == 0) {
        snprintf(error, n, "No se detectó ningún FT2232H.");
        Cleanup_libMPSSE();
        return false;
    }
    if (SPI_OpenChannel(daq_canal_a(canales), h) != FT_OK) {
        snprintf(error, n, "No se pudo abrir la interfaz A (¿está en uso?).");
        Cleanup_libMPSSE();
        return false;
    }

    ChannelConfig conf;
    memset(&conf, 0, sizeof conf);
    conf.ClockRate = reloj;
    conf.LatencyTimer = 1;
    conf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 |
                         SPI_CONFIG_OPTION_CS_ACTIVELOW;
    if (SPI_InitChannel(*h, &conf) != FT_OK) {
        snprintf(error, n, "Falló la inicialización del canal SPI.");
        SPI_CloseChannel(*h);
        Cleanup_libMPSSE();
        return false;
    }

#ifndef _WIN32
    if (lazo && SPI_Lazo(*h, true) != FT_OK) {
        snprintf(error, n, "No se pudo activar el lazo interno.");
        SPI_CloseChannel(*h);
        Cleanup_libMPSSE();
        return false;
    }
#else
    (void)lazo;
#endif
    return true;
}

/* Deja la salida en 0 V con una trama de reposo y cierra el canal */
static inline void enlace_cerrar(FT_HANDLE h, bool lazo, uint8_t seq)
{
    uint8_t tx[DAQ_TRAMA_LEN];
    uint8_t rx[DAQ_TRAMA_LEN];
    DWORD n = 0;

#ifndef _WIN32
    if (lazo) SPI_Lazo(h, false);
#else
    (void)lazo;
#endif
    daq_armar_trama_pc(tx, seq, DAQ_DAC_CERO, 0u);
    SPI_ReadWrite(h, rx, tx, DAQ_TRAMA_LEN, &n, kOpciones);
    SPI_CloseChannel(h);
    Cleanup_libMPSSE();
}

#endif /* ENLACE_PRUEBA_H */

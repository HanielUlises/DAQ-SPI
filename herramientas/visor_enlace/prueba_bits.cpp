/*
 * Pruebas de bits.h sin hardware. Las tramas se generan con el simulador del
 * dsPIC o se arman a mano, y se clasifican con prueba_procesar() igual que
 * en la adquisición.
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "bits.h"
#include "simulador.h"

static int fallas = 0;

#define VERIFICA(cond)                                                    \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FALLA %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
            fallas++;                                                     \
        }                                                                 \
    } while (0)

/* Banco de prueba: simulador + clasificación + análisis */
struct Banco {
    Simulador s;
    EstadoPrueba e;
    AnalisisBits a;
    unsigned long k = 0;
    bool lazo = false;

    explicit Banco(size_t max_malas = 200) : a(max_malas)
    {
        prueba_iniciar(&e);
        a.reiniciar(false);
    }

    /* Una transferencia; corromper, si no es NULL, modifica la respuesta */
    Trama paso(double voltaje = 0.0, bool salida = false,
               void (*corromper)(uint8_t *) = NULL, bool analizar = true)
    {
        Trama t;
        memset(&t, 0, sizeof t);
        t.k = k;
        t.t_us = 1000.0 * (double)k;
        t.dt_us = 120.0;
        daq_armar_trama_pc(t.tx, (uint8_t)k, daq_voltaje_a_dac(voltaje),
                           salida ? DAQ_FLAG_SALIDA_HAB : 0u);
        s.transferir(t.tx, t.rx, t.t_us);
        if (corromper != NULL) corromper(t.rx);
        prueba_procesar(&e, &t, FT_OK, DAQ_TRAMA_LEN, lazo);
        if (analizar) a.agregar(t);
        k++;
        return t;
    }
};

static unsigned long total_mapa(const AnalisisBits &a)
{
    unsigned long n = 0;
    for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) {
        for (unsigned b = 0; b < 8u; b++) n += a.mapa[i][b];
    }
    return n;
}

static void bit7_byte7(uint8_t *rx) { rx[7] ^= 0x80u; }
static void bit3_byte1(uint8_t *rx) { rx[1] ^= 0x08u; }
static void bit0_byte4(uint8_t *rx) { rx[4] ^= 0x01u; }
static void sin_encabezado(uint8_t *rx) { rx[0] = 0xFFu; }

/* Sin errores, con el motor quieto: nada distinto y ninguna trama mala */
static void prueba_limpia(void)
{
    Banco b;
    for (int i = 0; i < 1000; i++) b.paso();
    VERIFICA(b.a.analizadas == 999u);       /* k = 0 no se analiza */
    VERIFICA(b.a.distintas == 0u);
    VERIFICA(b.a.arranques == 0u);
    VERIFICA(b.a.num_malas() == 0u);
    VERIFICA(total_mapa(b.a) == 0u);
}

/* Un bit invertido en la respuesta se ubica en su byte y bit */
static void prueba_bit_aislado(void)
{
    Banco b;
    for (int i = 0; i < 100; i++) b.paso();
    Trama t = b.paso(0.0, false, bit7_byte7);
    VERIFICA(t.resultado == RES_CRC);
    for (int i = 0; i < 10; i++) b.paso();

    VERIFICA(b.a.distintas == 1u);
    VERIFICA(b.a.mapa[7][7] == 1u);
    VERIFICA(total_mapa(b.a) == 1u);
    VERIFICA(b.a.num_malas() == 1u);
    const TramaMala &m = b.a.mala(0);
    VERIFICA(m.t.k == 100u);
    VERIFICA(m.analizada);
    VERIFICA(m.diferencia[7] == 0x80u);
    VERIFICA(m.status_siguiente == 0);
}

/* Con el motor girando, la posición extrapolada evita bits falsos */
static void prueba_motor(void)
{
    Banco b;
    for (int i = 0; i < 600; i++) b.paso(1.0, true);       /* velocidad estable */
    VERIFICA(b.a.distintas == 0u);
    b.paso(1.0, true, bit3_byte1);
    b.paso(1.0, true);
    VERIFICA(b.a.distintas == 1u);
    VERIFICA(b.a.mapa[1][3] == 1u);
    /* La posición extrapolada puede diferir en una cuenta del valor real */
    VERIFICA(total_mapa(b.a) <= 3u);
}

/* Extrapolación exacta con posiciones armadas a mano */
static void prueba_extrapolacion(void)
{
    AnalisisBits a;
    a.reiniciar(false);
    uint8_t seq_anterior = 0;
    for (unsigned long k = 0; k < 20; k++) {
        Trama t;
        memset(&t, 0, sizeof t);
        t.k = k;
        daq_armar_trama_pc(t.tx, (uint8_t)k, DAQ_DAC_CERO, 0u);
        const int32_t pos = (int32_t)(1000 + 37 * (long)k);
        uint8_t r[DAQ_TRAMA_LEN];
        r[0] = DAQ_INICIO_PIC;
        r[1] = seq_anterior;
        for (int i = 0; i < 4; i++) r[2 + i] = (uint8_t)((uint32_t)pos >> (8 * i));
        r[6] = 0u;
        r[7] = daq_crc8(r, DAQ_TRAMA_LEN - 1u);
        memcpy(t.rx, r, DAQ_TRAMA_LEN);
        if (k == 15) t.rx[4] ^= 0x01u;      /* bit 0 del byte 2 de la posición */
        t.resultado = (k == 0) ? RES_PRIMERA : (k == 15 ? RES_CRC : RES_OK);
        t.status = (k == 15) ? -1 : 0;
        t.posicion = pos;
        a.agregar(t);
        seq_anterior = (uint8_t)k;
    }
    VERIFICA(a.distintas == 1u);
    VERIFICA(a.mapa[4][0] == 1u);
    VERIFICA(total_mapa(a) == 1u);
}

/* La bandera de longitud de una transferencia llega en la respuesta
 * siguiente y queda anotada en la trama mala */
static void prueba_status_siguiente(void)
{
    AnalisisBits a;
    a.reiniciar(false);
    Banco b;
    for (int i = 0; i < 10; i++) a.agregar(b.paso());

    Trama mala = b.paso(0.0, false, bit7_byte7, false);
    a.agregar(mala);
    Trama sig = b.paso(0.0, false, NULL, false);
    sig.status = DAQ_STATUS_ERR_LONGITUD;   /* como lo reportaría el dsPIC */
    a.agregar(sig);

    VERIFICA(a.num_malas() == 2u);          /* la del CRC y la del status */
    VERIFICA(a.mala(0).t.k == 10u);
    VERIFICA(a.mala(0).status_siguiente == (int)DAQ_STATUS_ERR_LONGITUD);
    VERIFICA(a.mala(1).t.k == 11u);
    VERIFICA(a.mala(1).status_siguiente == -1);     /* aún no llega la 12 */
}

/* Un reinicio del dsPIC: la respuesta de arranque se cuenta aparte y la
 * causa llega en la siguiente */
static void prueba_arranque(void)
{
    Banco b;
    for (int i = 0; i < 50; i++) b.paso();
    b.s.arrancar(DAQ_REINICIO_TRAMPA_DMA);
    Trama arranque = b.paso();
    Trama causa = b.paso();
    for (int i = 0; i < 5; i++) b.paso();

    VERIFICA(arranque.resultado == RES_SEQ);
    VERIFICA(b.a.arranques == 1u);
    VERIFICA(b.a.distintas == 0u);
    VERIFICA(((causa.status & DAQ_STATUS_REINICIO) >> DAQ_STATUS_REINICIO_POS) ==
             DAQ_REINICIO_TRAMPA_DMA);
    VERIFICA(b.a.num_malas() == 2u);
    VERIFICA(!b.a.mala(0).analizada);
    VERIFICA(b.a.mala(0).status_siguiente == causa.status);
    VERIFICA(b.a.mala(1).t.k == causa.k);
}

/* Un salto en k reinicia el contexto: esa trama no se analiza */
static void prueba_salto(void)
{
    Banco b;
    for (int i = 0; i < 20; i++) b.paso();
    for (int i = 0; i < 5; i++) b.paso(0.0, false, NULL, false);   /* descartadas */
    const unsigned long antes = b.a.analizadas;
    b.paso();
    VERIFICA(b.a.analizadas == antes);
    b.paso();
    VERIFICA(b.a.analizadas == antes + 1u);
    VERIFICA(b.a.distintas == 0u);
}

/* Respuestas sin encabezado: tramas malas sin analizar */
static void prueba_sin_encabezado(void)
{
    Banco b;
    for (int i = 0; i < 10; i++) b.paso();
    Trama t = b.paso(0.0, false, sin_encabezado);
    VERIFICA(t.resultado == RES_INICIO);
    VERIFICA(b.a.num_malas() == 1u);
    VERIFICA(!b.a.mala(0).analizada);
    VERIFICA(total_mapa(b.a) == 0u);
}

/* El historial guarda sólo las más recientes */
static void prueba_historial(void)
{
    Banco b(5);
    b.paso();
    for (int i = 0; i < 12; i++) {
        b.paso(0.0, false, bit0_byte4);
        b.paso();
        b.paso();
    }
    VERIFICA(b.a.num_malas() == 5u);
    VERIFICA(b.a.mala(0).t.k == 1u + 3u * 7u);
    VERIFICA(b.a.mala(4).t.k == 1u + 3u * 11u);
    VERIFICA(b.a.distintas == 12u);
    VERIFICA(b.a.mapa[4][0] == 12u);
}

/* En lazo, la esperada es la trama enviada */
static void prueba_lazo(void)
{
    AnalisisBits a;
    a.reiniciar(true);
    EstadoPrueba e;
    prueba_iniciar(&e);
    for (unsigned long k = 0; k < 100; k++) {
        Trama t;
        memset(&t, 0, sizeof t);
        t.k = k;
        daq_armar_trama_pc(t.tx, (uint8_t)k, aleatorio(), 0u);
        memcpy(t.rx, t.tx, DAQ_TRAMA_LEN);
        if (k == 40) t.rx[3] ^= 0x10u;
        prueba_procesar(&e, &t, FT_OK, DAQ_TRAMA_LEN, true);
        a.agregar(t);
    }
    VERIFICA(a.analizadas == 100u);
    VERIFICA(a.distintas == 1u);
    VERIFICA(a.mapa[3][4] == 1u);
    VERIFICA(a.num_malas() == 1u);
    VERIFICA(a.mala(0).t.resultado == RES_ECO);
}

/* Con errores al azar en ambos sentidos, el mapa recupera la tasa de bits
 * invertidos en la respuesta y los reparte en todas las posiciones */
static void prueba_ber(void)
{
    Banco b;
    const double ber = 1e-3;
    const unsigned long n = 50000;
    b.s.ber = ber;
    for (unsigned long i = 0; i < n; i++) b.paso();

    /* Una respuesta con el byte 0 dañado no se analiza: sólo cuentan los
     * bits invertidos en los bytes 1 a 7 */
    const double esperado = ber * 56.0 * (double)b.a.analizadas;
    const double medido = (double)total_mapa(b.a);
    printf("  BER: %lu bits en el mapa, %.0f esperados por la tasa\n",
           total_mapa(b.a), esperado);
    VERIFICA(medido > 0.9 * esperado && medido < 1.1 * esperado);
    for (unsigned i = 1; i < DAQ_TRAMA_LEN; i++) {
        unsigned long fila = 0;
        for (unsigned bit = 0; bit < 8u; bit++) fila += b.a.mapa[i][bit];
        VERIFICA(fila > 0u);
    }
}

/* Con el motor acelerando (rampa de +/-1 V) y errores al azar, la posición
 * esperada no debe inflar las filas de posición ni la del CRC */
static void prueba_ber_motor(void)
{
    Banco b;
    const double ber = 2e-4;
    const unsigned long n = 50000;
    b.s.ber = ber;
    for (unsigned long i = 0; i < n; i++) {
        const double fase = (double)(i % 2000u) / 2000.0;
        const double v = fase < 0.5 ? -1.0 + 4.0 * fase : 3.0 - 4.0 * fase;
        b.paso(v, true);
    }
    const double esperado = ber * 56.0 * (double)b.a.analizadas;
    const double medido = (double)total_mapa(b.a);
    unsigned long pos_crc = 0;
    for (unsigned i = DAQ_PIC_ENC0; i < DAQ_TRAMA_LEN; i++) {
        if (i == DAQ_PIC_STATUS) continue;
        for (unsigned bit = 0; bit < 8u; bit++) pos_crc += b.a.mapa[i][bit];
    }
    printf("  BER con motor: %lu bits en el mapa (%lu en posición y CRC), %.0f esperados\n",
           total_mapa(b.a), pos_crc, esperado);
    VERIFICA(medido > 0.9 * esperado && medido < 1.1 * esperado);
    /* Posición (4 bytes) y CRC: 5 de los 7 bytes analizables */
    VERIFICA((double)pos_crc < 1.15 * esperado * 5.0 / 7.0);
}

int main(void)
{
    prueba_limpia();
    prueba_bit_aislado();
    prueba_motor();
    prueba_extrapolacion();
    prueba_status_siguiente();
    prueba_arranque();
    prueba_salto();
    prueba_sin_encabezado();
    prueba_historial();
    prueba_lazo();
    prueba_ber();
    prueba_ber_motor();

    if (fallas == 0) {
        printf("OK: todas las pruebas de bits.h pasaron\n");
        return 0;
    }
    printf("%d fallas\n", fallas);
    return 1;
}

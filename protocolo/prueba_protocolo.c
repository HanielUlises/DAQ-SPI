/*
 * Pruebas en la PC del protocolo DAQ-SPI (sin hardware).
 *
 *   cc -std=c99 -Wall -Wextra -o prueba_protocolo prueba_protocolo.c
 *   ./prueba_protocolo
 *
 * Incluye un modelo del esclavo que reproduce la lógica de enlace.c: la
 * respuesta de la transferencia k se construye al terminar la k-1.
 */
#include <stdio.h>
#include <string.h>
#include "daq_protocolo.h"

static int fallas = 0;

#define VERIFICA(cond)                                                    \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FALLA %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
            fallas++;                                                     \
        }                                                                 \
    } while (0)

/* ---- Modelo del dsPIC ---------------------------------------------------- */

struct esclavo {
    uint8_t tx[DAQ_TRAMA_LEN];
    uint8_t seq_eco;
    uint8_t status_pendiente;
    int32_t posicion;
    uint16_t dac;
    int en_falla;
};

static void esclavo_construir_respuesta(struct esclavo *e)
{
    uint32_t pos = (uint32_t)e->posicion;

    e->tx[DAQ_PIC_INICIO]   = DAQ_INICIO_PIC;
    e->tx[DAQ_PIC_SEQ]      = e->seq_eco;
    e->tx[DAQ_PIC_ENC0]     = (uint8_t)pos;
    e->tx[DAQ_PIC_ENC0 + 1] = (uint8_t)(pos >> 8);
    e->tx[DAQ_PIC_ENC0 + 2] = (uint8_t)(pos >> 16);
    e->tx[DAQ_PIC_ENC0 + 3] = (uint8_t)(pos >> 24);
    e->tx[DAQ_PIC_STATUS]   = e->status_pendiente;
    e->tx[DAQ_PIC_CRC]      = daq_crc8(e->tx, DAQ_TRAMA_LEN - 1u);
    e->status_pendiente = 0u;
}

/* Transferencia full-duplex de n bytes seguida del flanco de subida de CS */
static void esclavo_transferir(struct esclavo *e, const uint8_t *mosi, uint8_t *miso,
                               unsigned n)
{
    unsigned i;

    for (i = 0; i < n; i++) {
        miso[i] = (i < DAQ_TRAMA_LEN) ? e->tx[i] : 0u;
    }

    if (n != DAQ_TRAMA_LEN) {
        e->status_pendiente |= DAQ_STATUS_ERR_LONGITUD;
    } else if (mosi[DAQ_PC_INICIO] != DAQ_INICIO_PC) {
        e->status_pendiente |= DAQ_STATUS_ERR_INICIO;
    } else if (daq_crc8(mosi, DAQ_TRAMA_LEN - 1u) != mosi[DAQ_PC_CRC]) {
        e->status_pendiente |= DAQ_STATUS_ERR_CRC;
    } else {
        e->seq_eco = mosi[DAQ_PC_SEQ];
        if (mosi[DAQ_PC_FLAGS] & DAQ_FLAG_RESET_ENC) {
            e->posicion = 0;
        }
        e->dac = (uint16_t)((mosi[DAQ_PC_FLAGS] & DAQ_FLAG_SALIDA_HAB)
                                ? ((unsigned)mosi[DAQ_PC_DAC_H] << 8) | mosi[DAQ_PC_DAC_L]
                                : DAQ_DAC_CERO);
        e->en_falla = 0;    /* trama válida: se reinicia la vigilancia */
    }
    esclavo_construir_respuesta(e);
}

/* Expira la vigilancia (_T1Interrupt): pasaron 50 ms sin una trama válida.
 * La respuesta ya armada no cambia; la bandera sale en la siguiente. */
static void esclavo_vigilancia(struct esclavo *e)
{
    if (!e->en_falla) {
        e->en_falla = 1;
        e->status_pendiente |= DAQ_STATUS_VIGILANCIA;
        e->dac = DAQ_DAC_CERO;
    }
}

/* Arranque (enlace_inicializar): la respuesta inicial no lleva la causa del
 * reinicio; ésta sale en la primera respuesta que arma la interrupción. */
static void esclavo_arrancar(struct esclavo *e, uint8_t causa)
{
    memset(e, 0, sizeof *e);
    e->en_falla = 1;
    esclavo_construir_respuesta(e);
    e->status_pendiente = (uint8_t)(causa << DAQ_STATUS_REINICIO_POS);
}

/* ---- Pruebas ------------------------------------------------------------- */

static void prueba_crc(void)
{
    const uint8_t ref[] = "123456789";
    uint8_t t[DAQ_TRAMA_LEN];
    unsigned byte;
    unsigned bit;

    VERIFICA(daq_crc8(ref, 9) == 0xF4u);
    VERIFICA(daq_crc8(ref, 0) == 0x00u);

    /* Todo error de un bit en los bytes 0 a 7 se detecta */
    daq_armar_trama_pc(t, 0x42u, 0x1234u, DAQ_FLAG_SALIDA_HAB);
    for (byte = 0; byte < DAQ_TRAMA_LEN; byte++) {
        for (bit = 0; bit < 8; bit++) {
            t[byte] ^= (uint8_t)(1u << bit);
            VERIFICA(daq_crc8(t, DAQ_TRAMA_LEN - 1u) != t[DAQ_PC_CRC]);
            t[byte] ^= (uint8_t)(1u << bit);
        }
    }
}

static void prueba_trama_pc(void)
{
    uint8_t t[DAQ_TRAMA_LEN];

    daq_armar_trama_pc(t, 7u, 0xA55Au, DAQ_FLAG_SALIDA_HAB | DAQ_FLAG_RESET_ENC);
    VERIFICA(t[DAQ_PC_INICIO] == 0xA5u);
    VERIFICA(t[DAQ_PC_SEQ] == 7u);
    VERIFICA(t[DAQ_PC_DAC_H] == 0xA5u);
    VERIFICA(t[DAQ_PC_DAC_L] == 0x5Au);
    VERIFICA(t[DAQ_PC_FLAGS] == 0x03u);
    VERIFICA(t[5] == 0u && t[6] == 0u);
    VERIFICA(t[DAQ_PC_CRC] == daq_crc8(t, 7u));
}

static void prueba_voltaje(void)
{
    const double nan = 0.0 / 0.0;

    VERIFICA(daq_voltaje_a_dac(-2.5) == 0u);
    VERIFICA(daq_voltaje_a_dac(0.0) == DAQ_DAC_CERO);
    VERIFICA(daq_voltaje_a_dac(2.5) == 65535u);
    VERIFICA(daq_voltaje_a_dac(1.0) == 45874u);

    /* Saturación y NaN */
    VERIFICA(daq_voltaje_a_dac(-10.0) == 0u);
    VERIFICA(daq_voltaje_a_dac(10.0) == 65535u);
    VERIFICA(daq_voltaje_a_dac(nan) == DAQ_DAC_CERO);
}

static void prueba_trama_pic(void)
{
    struct esclavo e;
    uint8_t seq;
    int32_t pos;
    uint8_t status;
    const int32_t valores[] = { 0, 1, -1, 123456, -123456, 2147483647, (-2147483647 - 1) };
    unsigned i;

    memset(&e, 0, sizeof e);
    for (i = 0; i < sizeof valores / sizeof valores[0]; i++) {
        e.posicion = valores[i];
        e.seq_eco = (uint8_t)i;
        e.status_pendiente = DAQ_STATUS_ERR_CRC;
        esclavo_construir_respuesta(&e);
        VERIFICA(daq_leer_trama_pic(e.tx, &seq, &pos, &status) == 0);
        VERIFICA(pos == valores[i]);
        VERIFICA(seq == (uint8_t)i);
        VERIFICA(status == DAQ_STATUS_ERR_CRC);
    }

    e.tx[DAQ_PIC_ENC0] ^= 0x01u;
    VERIFICA(daq_leer_trama_pic(e.tx, &seq, &pos, &status) == -2);
    e.tx[DAQ_PIC_INICIO] = 0xFFu;
    VERIFICA(daq_leer_trama_pic(e.tx, &seq, &pos, &status) == -1);
}

/* Secuencia completa: el eco y la posición corresponden a la trama anterior */
static void prueba_secuencia(void)
{
    struct esclavo e;
    uint8_t mosi[DAQ_TRAMA_LEN + 2];
    uint8_t miso[DAQ_TRAMA_LEN + 2];
    uint8_t seq;
    int32_t pos;
    uint8_t status;
    unsigned k;

    memset(&e, 0, sizeof e);
    esclavo_construir_respuesta(&e);

    for (k = 0; k < 600; k++) {
        e.posicion = (int32_t)(k * 10);    /* el encoder avanza entre tramas */
        daq_armar_trama_pc(mosi, (uint8_t)k, (uint16_t)(k * 97u), DAQ_FLAG_SALIDA_HAB);
        esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);

        VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
        VERIFICA(status == 0u);
        VERIFICA(e.dac == (uint16_t)(k * 97u));
        if (k > 0) {
            VERIFICA(seq == (uint8_t)(k - 1u));
            /* posición muestreada al terminar la transferencia k-1 */
            VERIFICA(pos == (int32_t)((k - 1u) * 10));
        }
    }

    /* Trama con CRC incorrecto: el DAC no cambia y se reporta en la siguiente */
    daq_armar_trama_pc(mosi, 0x10u, 0x1111u, DAQ_FLAG_SALIDA_HAB);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    mosi[DAQ_PC_DAC_L] ^= 0x01u;
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(e.dac == 0x1111u);
    daq_armar_trama_pc(mosi, 0x12u, 0x2222u, DAQ_FLAG_SALIDA_HAB);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
    VERIFICA(status == DAQ_STATUS_ERR_CRC);
    VERIFICA(seq == 0x10u);     /* eco de la última trama válida */

    /* Trama de longitud incorrecta */
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN + 2u);
    daq_armar_trama_pc(mosi, 0x13u, 0x2222u, DAQ_FLAG_SALIDA_HAB);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
    VERIFICA(status == DAQ_STATUS_ERR_LONGITUD);

    /* Salida deshabilitada: el DAC va a 0 V */
    daq_armar_trama_pc(mosi, 0x14u, 0xFFFFu, 0u);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(e.dac == DAQ_DAC_CERO);

    /* Puesta en cero del encoder: se refleja en la respuesta siguiente */
    e.posicion = 5000;
    esclavo_construir_respuesta(&e);
    daq_armar_trama_pc(mosi, 0x15u, 0u, DAQ_FLAG_RESET_ENC);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
    VERIFICA(pos == 5000);
    daq_armar_trama_pc(mosi, 0x16u, 0u, 0u);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
    VERIFICA(pos == 0);
}

/* Cada respuesta reporta los errores ocurridos desde la anterior */
static void prueba_errores_acumulados(void)
{
    struct esclavo e;
    uint8_t mosi[DAQ_TRAMA_LEN];
    uint8_t miso[DAQ_TRAMA_LEN + 2];
    uint8_t seq;
    int32_t pos;
    uint8_t status;

    memset(&e, 0, sizeof e);
    esclavo_construir_respuesta(&e);

    /* Error de inicio: se reporta en la respuesta siguiente */
    daq_armar_trama_pc(mosi, 1u, 0x1000u, DAQ_FLAG_SALIDA_HAB);
    mosi[DAQ_PC_INICIO] = 0x00u;
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(e.tx, &seq, &pos, &status) == 0);
    VERIFICA(status == DAQ_STATUS_ERR_INICIO);

    /* La vigilancia expira y luego llega una trama corta: ambos en un status */
    esclavo_vigilancia(&e);
    esclavo_transferir(&e, mosi, miso, 3u);
    VERIFICA(daq_leer_trama_pic(e.tx, &seq, &pos, &status) == 0);
    VERIFICA(status == (DAQ_STATUS_VIGILANCIA | DAQ_STATUS_ERR_LONGITUD));

    /* Tras reportarse, el status se limpia */
    daq_armar_trama_pc(mosi, 2u, 0x1000u, DAQ_FLAG_SALIDA_HAB);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(e.tx, &seq, &pos, &status) == 0);
    VERIFICA(status == 0u);
    VERIFICA(seq == 2u);
    VERIFICA(e.dac == 0x1000u);
}

/* El número de secuencia da la vuelta de 255 a 0 sin errores */
static void prueba_seq_vuelta(void)
{
    struct esclavo e;
    uint8_t mosi[DAQ_TRAMA_LEN];
    uint8_t miso[DAQ_TRAMA_LEN];
    uint8_t seq = 0u;
    int32_t pos;
    uint8_t status;
    uint8_t esperado = 0u;
    unsigned k;

    memset(&e, 0, sizeof e);
    esclavo_construir_respuesta(&e);

    for (k = 250u; k < 262u; k++) {
        daq_armar_trama_pc(mosi, (uint8_t)k, 0u, 0u);
        esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
        VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
        if (k > 250u) {
            VERIFICA(seq == esperado);
        }
        esperado = (uint8_t)k;
    }
    VERIFICA(e.seq_eco == (uint8_t)261u);
}

static void prueba_vigilancia(void)
{
    struct esclavo e;
    uint8_t mosi[DAQ_TRAMA_LEN];
    uint8_t miso[DAQ_TRAMA_LEN];
    uint8_t seq;
    int32_t pos;
    uint8_t status;

    memset(&e, 0, sizeof e);
    esclavo_construir_respuesta(&e);

    daq_armar_trama_pc(mosi, 1u, 60000u, DAQ_FLAG_SALIDA_HAB);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(e.dac == 60000u);

    /* La PC deja de enviar: el DAC va a 0 V y la bandera se levanta una vez */
    esclavo_vigilancia(&e);
    VERIFICA(e.dac == DAQ_DAC_CERO);
    esclavo_vigilancia(&e);
    esclavo_construir_respuesta(&e);
    VERIFICA(daq_leer_trama_pic(e.tx, &seq, &pos, &status) == 0);
    VERIFICA(status == DAQ_STATUS_VIGILANCIA);
    esclavo_construir_respuesta(&e);
    VERIFICA(daq_leer_trama_pic(e.tx, &seq, &pos, &status) == 0);
    VERIFICA(status == 0u);

    /* Una trama inválida no saca al esclavo de la falla */
    daq_armar_trama_pc(mosi, 2u, 50000u, DAQ_FLAG_SALIDA_HAB);
    mosi[DAQ_PC_CRC] ^= 0xFFu;
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(e.dac == DAQ_DAC_CERO);
    VERIFICA(e.en_falla);

    /* La comunicación se recupera con la siguiente trama válida */
    daq_armar_trama_pc(mosi, 3u, 50000u, DAQ_FLAG_SALIDA_HAB);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(e.dac == 50000u);
    VERIFICA(!e.en_falla);

    /* Tras recuperarse, una nueva expiración se reporta otra vez */
    esclavo_vigilancia(&e);
    esclavo_construir_respuesta(&e);
    VERIFICA(daq_leer_trama_pic(e.tx, &seq, &pos, &status) == 0);
    VERIFICA(status == DAQ_STATUS_VIGILANCIA);
    VERIFICA(seq == 3u);
}

static void prueba_reinicio(void)
{
    struct esclavo e;
    uint8_t mosi[DAQ_TRAMA_LEN];
    uint8_t miso[DAQ_TRAMA_LEN];
    uint8_t seq;
    int32_t pos;
    uint8_t status;

    /* El campo de reinicio no se encima con las banderas de error */
    VERIFICA((DAQ_STATUS_REINICIO & (DAQ_STATUS_ERR_CRC | DAQ_STATUS_ERR_INICIO |
                                     DAQ_STATUS_VIGILANCIA | DAQ_STATUS_ERR_LONGITUD)) == 0u);
    VERIFICA((DAQ_REINICIO_DESCONOCIDO << DAQ_STATUS_REINICIO_POS) == DAQ_STATUS_REINICIO);

    esclavo_arrancar(&e, DAQ_REINICIO_TRAMPA_DMA);

    /* La respuesta inicial es la de arranque, sin causa */
    daq_armar_trama_pc(mosi, 7u, DAQ_DAC_CERO, 0u);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
    VERIFICA(seq == 0u);
    VERIFICA(status == 0u);

    /* La siguiente trae la causa junto con las banderas de error pendientes */
    daq_armar_trama_pc(mosi, 8u, DAQ_DAC_CERO, 0u);
    mosi[DAQ_PC_CRC] ^= 0xFFu;
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
    VERIFICA(seq == 7u);
    VERIFICA((status & DAQ_STATUS_REINICIO) >> DAQ_STATUS_REINICIO_POS == DAQ_REINICIO_TRAMPA_DMA);
    VERIFICA((status & ~DAQ_STATUS_REINICIO) == 0u);

    /* Se reporta una sola vez */
    daq_armar_trama_pc(mosi, 9u, DAQ_DAC_CERO, 0u);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
    VERIFICA(status == DAQ_STATUS_ERR_CRC);
    daq_armar_trama_pc(mosi, 10u, DAQ_DAC_CERO, 0u);
    esclavo_transferir(&e, mosi, miso, DAQ_TRAMA_LEN);
    VERIFICA(daq_leer_trama_pic(miso, &seq, &pos, &status) == 0);
    VERIFICA(status == 0u);
    VERIFICA(seq == 9u);
}

int main(void)
{
    prueba_crc();
    prueba_trama_pc();
    prueba_voltaje();
    prueba_trama_pic();
    prueba_secuencia();
    prueba_errores_acumulados();
    prueba_seq_vuelta();
    prueba_vigilancia();
    prueba_reinicio();

    if (fallas == 0) {
        printf("OK: todas las pruebas pasaron\n");
        return 0;
    }
    printf("%d fallas\n", fallas);
    return 1;
}

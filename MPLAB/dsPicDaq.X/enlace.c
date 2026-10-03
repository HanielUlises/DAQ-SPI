/*
 * Enlace SPI con la PC.
 *
 * Cada paso de simulación la PC ejecuta una sola transferencia full-duplex de
 * DAQ_TRAMA_LEN bytes. Durante la transferencia el DMA mueve los datos entre
 * SPI1 y la RAM sin intervención de la CPU. Al subir CS, la interrupción INT1
 * valida la trama recibida, reinicia SPI1, construye la respuesta siguiente y
 * vuelve a programar el DMA.
 *
 * SPI1 opera en modo de buffer estándar (ENHBUF = 0): cada byte recibido
 * genera el evento SPIRBF, que dispara el canal de RX, y cada vez que el
 * buffer de transmisión queda vacío se genera SPITBE, que dispara el canal
 * de TX. La FIFO del modo mejorado (4 niveles en 8 bits) no alcanza para una
 * trama completa, por lo que no aporta en este esquema.
 *
 * Referencias: DS70005399C, tabla 10-1 (disparos de DMA) y sección 16 (SPI).
 */
#include <xc.h>
#include "enlace.h"
#include "mcc_generated_files/system/pins.h"
#include "../../protocolo/daq_protocolo.h"

/* Disparos de DMA (DS70005399C, tabla 10-1) */
#define DMA_CHSEL_SPI1_RX       0x02u
#define DMA_CHSEL_SPI1_TX       0x03u

/* Límites de la RAM de datos para el DMA (8 KB a partir de 0x1000) */
#define DMA_RAM_INICIO          0x1000u
#define DMA_RAM_FIN             0x2FFFu

/* PPS: RB10 corresponde a RP42 */
#define PPS_RP_RB10             42u

/* Vigilancia: FP = 4 MHz, preescalador 1:64 -> 62.5 kHz; 50 ms = 3125 cuentas */
#define VIGILANCIA_PR1          3124u

#define PRIORIDAD_ENLACE        5u

/* Buffers de DMA. Se usan palabras de 16 bits para que cada transferencia
 * lea o escriba el registro SPI1BUFL completo; solo el byte bajo es útil. */
static uint16_t rx_dma[DAQ_TRAMA_LEN];
static uint16_t tx_dma[DAQ_TRAMA_LEN];

/* Estado compartido con el lazo principal */
static volatile uint16_t dac_valor = DAQ_DAC_CERO;
static volatile bool dac_nuevo = false;

/* Estado privado de las interrupciones (misma prioridad, no se anidan) */
static uint8_t seq_eco = 0u;
static uint8_t status_pendiente = 0u;
static bool en_falla = true;

static int32_t leer_posicion(void)
{
    uint16_t baja;
    uint16_t alta;

    /* La lectura de POS1CNTL copia la parte alta en POS1HLD; el orden de
     * acceso debe ser explícito. */
    baja = POS1CNTL;
    alta = POS1HLD;
    return (int32_t)(((uint32_t)alta << 16) | baja);
}

static void construir_respuesta(void)
{
    uint8_t trama[DAQ_TRAMA_LEN];
    uint32_t pos = (uint32_t)leer_posicion();
    uint8_t i;

    trama[DAQ_PIC_INICIO]   = DAQ_INICIO_PIC;
    trama[DAQ_PIC_SEQ]      = seq_eco;
    trama[DAQ_PIC_ENC0]     = (uint8_t)pos;
    trama[DAQ_PIC_ENC0 + 1] = (uint8_t)(pos >> 8);
    trama[DAQ_PIC_ENC0 + 2] = (uint8_t)(pos >> 16);
    trama[DAQ_PIC_ENC0 + 3] = (uint8_t)(pos >> 24);
    trama[DAQ_PIC_STATUS]   = status_pendiente;
    trama[DAQ_PIC_CRC]      = daq_crc8(trama, DAQ_TRAMA_LEN - 1u);
    status_pendiente = 0u;

    for (i = 0u; i < DAQ_TRAMA_LEN; i++) {
        tx_dma[i] = trama[i];
    }
}

/* Reinicia SPI1. Con SPIEN = 0 el módulo se reinicia: se vacían los buffers y
 * se borran SPIROV y SPITUR, de modo que cada trama comienza alineada. */
static void reiniciar_spi1(void)
{
    SPI1CON1Lbits.SPIEN = 0;
    SPI1STATLbits.SPIROV = 0;
    SPI1CON1Lbits.SPIEN = 1;
}

/* Programa ambos canales para la siguiente trama. Requiere SPI1 habilitado y
 * CS en alto (sin actividad en el bus). */
static void armar_dma(void)
{
    /* DMA0: SPI1BUFL -> rx_dma[], una palabra por cada byte recibido */
    DMACH0 = 0u;
    DMASRC0 = (uint16_t)&SPI1BUFL;
    DMADST0 = (uint16_t)rx_dma;
    DMACNT0 = DAQ_TRAMA_LEN;
    DMAINT0 = (uint16_t)(DMA_CHSEL_SPI1_RX << 8);
    DMACH0bits.DAMODE = 1;      /* destino con incremento */
    DMACH0bits.CHEN = 1;

    /* El primer byte se escribe directamente; el DMA envía el resto. */
    SPI1BUFL = tx_dma[0];

    /* DMA1: tx_dma[1..] -> SPI1BUFL, una palabra por cada evento SPITBE */
    DMACH1 = 0u;
    DMASRC1 = (uint16_t)&tx_dma[1];
    DMADST1 = (uint16_t)&SPI1BUFL;
    DMACNT1 = DAQ_TRAMA_LEN - 1u;
    DMAINT1 = (uint16_t)(DMA_CHSEL_SPI1_TX << 8);
    DMACH1bits.SAMODE = 1;      /* origen con incremento */
    DMACH1bits.CHEN = 1;

    /* Si el primer byte ya pasó al registro de corrimiento antes de habilitar
     * el canal, el evento SPITBE se perdió: se solicita la transferencia por
     * software. Con CS en alto el estado del módulo no cambia. */
    Nop();
    Nop();
    if (SPI1STATLbits.SPITBE && (DMACNT1 == (DAQ_TRAMA_LEN - 1u))) {
        DMACH1bits.CHREQ = 1;
    }
}

static void procesar_trama(void)
{
    uint8_t trama[DAQ_TRAMA_LEN];
    bool completa;
    uint16_t dac;
    uint8_t flags;
    uint8_t i;

    /* Exactamente DAQ_TRAMA_LEN bytes: el canal de RX terminó y no quedaron
     * bytes adicionales en el buffer. */
    completa = DMAINT0bits.DONEIF && SPI1STATLbits.SPIRBE && !SPI1STATLbits.SPIROV;

    if (!completa) {
        status_pendiente |= DAQ_STATUS_ERR_LONGITUD;
        return;
    }

    for (i = 0u; i < DAQ_TRAMA_LEN; i++) {
        trama[i] = (uint8_t)rx_dma[i];
    }

    if (trama[DAQ_PC_INICIO] != DAQ_INICIO_PC) {
        status_pendiente |= DAQ_STATUS_ERR_INICIO;
        return;
    }
    if (daq_crc8(trama, DAQ_TRAMA_LEN - 1u) != trama[DAQ_PC_CRC]) {
        status_pendiente |= DAQ_STATUS_ERR_CRC;
        return;
    }

    seq_eco = trama[DAQ_PC_SEQ];
    flags = trama[DAQ_PC_FLAGS];
    dac = ((uint16_t)trama[DAQ_PC_DAC_H] << 8) | trama[DAQ_PC_DAC_L];

    if (flags & DAQ_FLAG_RESET_ENC) {
        POS1HLD = 0u;
        POS1CNTL = 0u;
    }

    dac_valor = (flags & DAQ_FLAG_SALIDA_HAB) ? dac : DAQ_DAC_CERO;
    dac_nuevo = true;

    /* Trama válida: se reinicia la vigilancia */
    TMR1 = 0u;
    en_falla = false;
}

/* Fin de trama: flanco de subida de CS. El LED (RD10) permanece encendido
 * mientras se ejecuta la rutina, lo que permite medir su duración con un
 * analizador lógico; durante la comunicación el LED brilla de forma tenue. */
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void)
{
    IO_LED_SetHigh();
    _INT1IF = 0;

    DMACH0bits.CHEN = 0;
    DMACH1bits.CHEN = 0;

    procesar_trama();
    reiniciar_spi1();
    construir_respuesta();
    armar_dma();

    IO_LED_SetLow();
}

/* Vigilancia: no se recibió una trama válida en 50 ms */
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    _T1IF = 0;

    if (!en_falla) {
        en_falla = true;
        status_pendiente |= DAQ_STATUS_VIGILANCIA;
        dac_valor = DAQ_DAC_CERO;
        dac_nuevo = true;
    }
}

void enlace_inicializar(void)
{
    /* SPI1 esclavo, modo 0 (CKP = 0, CKE = 1), SS1 habilitado, buffer
     * estándar. La configuración de MCC se reemplaza aquí. */
    SPI1CON1Lbits.SPIEN = 0;
    SPI1CON1H = 0u;
    SPI1CON2L = 0u;
    SPI1STATL = 0u;
    SPI1CON1L = 0u;
    SPI1CON1Lbits.CKE = 1;
    SPI1CON1Lbits.SSEN = 1;

    /* Eventos que disparan el DMA. Las interrupciones de CPU de SPI1 quedan
     * deshabilitadas: los eventos solo sirven como disparo. */
    SPI1IMSKH = 0u;
    SPI1IMSKL = 0u;
    SPI1IMSKLbits.SPIRBFEN = 1;
    SPI1IMSKLbits.SPITBEN = 1;
    _SPI1RXIE = 0;
    _SPI1TXIE = 0;

    /* Controlador DMA, prioridad fija: DMA0 (RX) sobre DMA1 (TX) */
    DMACON = 0u;
    DMAL = DMA_RAM_INICIO;
    DMAH = DMA_RAM_FIN;
    DMACONbits.DMAEN = 1;

    /* INT1 en RB10 (mismo pin que SS1), flanco de subida */
    __builtin_write_RPCON(0x0000);
    RPINR0bits.INT1R = PPS_RP_RB10;
    __builtin_write_RPCON(0x0800);
    INTCON2bits.INT1EP = 0;
    _INT1IP = PRIORIDAD_ENLACE;

    /* Timer1 como temporizador de vigilancia */
    T1CON = 0u;
    T1CONbits.TCKPS = 2;        /* 1:64 */
    TMR1 = 0u;
    PR1 = VIGILANCIA_PR1;
    _T1IP = PRIORIDAD_ENLACE;
    _T1IF = 0;
    _T1IE = 1;
    T1CONbits.TON = 1;

    SPI1CON1Lbits.SPIEN = 1;
    construir_respuesta();
    armar_dma();

    _INT1IF = 0;
    _INT1IE = 1;
}

bool enlace_dac_pendiente(uint16_t *valor)
{
    if (!dac_nuevo) {
        return false;
    }
    /* Si la interrupción escribe un valor nuevo entre estas dos líneas, se
     * lee el valor nuevo y se reporta una vez más; no se pierde. */
    dac_nuevo = false;
    *valor = dac_valor;
    return true;
}

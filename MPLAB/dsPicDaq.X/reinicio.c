/*
 * Causa del último reinicio.
 *
 * Los manejadores de trampas que genera MCC (traps.c) llaman a
 * TRAPS_halt_on_error, que en compilación de producción ejecuta la
 * instrucción reset. Esa función es débil: aquí se sustituye por una que,
 * antes del reset, guarda el código de la trampa en RAM persistente (no la
 * borra el arranque de C), de modo que al arrancar se distingue un reset por
 * trampa de uno por software.
 */
#include <xc.h>
#include <stdbool.h>
#include "reinicio.h"
#include "mcc_generated_files/system/traps.h"
#include "../../protocolo/daq_protocolo.h"

#define TRAMPA_FIRMA    0xA5C3u

static volatile uint16_t trampa_firma __attribute__((persistent));
static volatile uint16_t trampa_codigo __attribute__((persistent));

static uint8_t causa = 0u;

void TRAPS_halt_on_error(uint16_t code)
{
    trampa_codigo = code;
    trampa_firma = TRAMPA_FIRMA;
#ifdef __DEBUG
    __builtin_software_breakpoint();
    while (1) {
    }
#else
    __asm__ volatile ("reset");
#endif
}

static uint8_t causa_trampa(uint16_t codigo)
{
    switch (codigo) {
    case TRAPS_OSC_FAIL:    return DAQ_REINICIO_TRAMPA_OSC;
    case TRAPS_STACK_ERR:   return DAQ_REINICIO_TRAMPA_PILA;
    case TRAPS_ADDRESS_ERR: return DAQ_REINICIO_TRAMPA_DIR;
    case TRAPS_MATH_ERR:    return DAQ_REINICIO_TRAMPA_MAT;
    case TRAPS_DAE_ERR:     return DAQ_REINICIO_TRAMPA_DMA;
    case TRAPS_HARD_ERR:    return DAQ_REINICIO_TRAMPA_HARD;
    default:                return DAQ_REINICIO_TRAMPA_OTRA;
    }
}

void reinicio_registrar(void)
{
    const RCONBITS bits = RCONbits;
    const bool trampa = !bits.POR && trampa_firma == TRAMPA_FIRMA;

    if (trampa && bits.SWR) {
        causa = causa_trampa(trampa_codigo);
    } else if (bits.TRAPR) {
        causa = DAQ_REINICIO_TRAMPA_OTRA;
    } else if (bits.IOPUWR) {
        causa = DAQ_REINICIO_OPCODE;
    } else if (bits.CM) {
        causa = DAQ_REINICIO_CONFIGURACION;
    } else if (bits.WDTO) {
        causa = DAQ_REINICIO_WATCHDOG;
    } else if (bits.SWR) {
        causa = DAQ_REINICIO_SOFTWARE;
    } else if (bits.EXTR) {
        causa = DAQ_REINICIO_MCLR;
    } else if (bits.POR) {
        causa = DAQ_REINICIO_ENCENDIDO;
    } else if (bits.BOR) {
        causa = DAQ_REINICIO_BAJO_VOLTAJE;
    } else {
        causa = DAQ_REINICIO_DESCONOCIDO;
    }

    RCON = 0u;
    trampa_firma = 0u;
}

uint8_t reinicio_causa(void)
{
    return causa;
}

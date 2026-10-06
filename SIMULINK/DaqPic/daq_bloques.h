/*
 * Estado compartido por los bloques daqPicInicio, daqPicEscribir y
 * daqPicLeer, la versión en tres bloques de daqPic.
 *
 * Sólo daqPicInicio usa el FT2232H: una vez por paso, en su mdlUpdate,
 * ejecuta la transferencia de DAQ_TRAMA_LEN bytes con el voltaje que dejó
 * daqPicEscribir y guarda la posición que entrega daqPicLeer. Simulink ejecuta
 * mdlOutputs de todos los bloques antes que cualquier mdlUpdate, de modo que
 * el voltaje del paso ya está escrito cuando se transfiere, sin importar el
 * orden en que se ejecuten los bloques. Como en daqPic, hay una sola
 * transferencia por paso y la posición llega con un retardo de dos periodos.
 *
 * Cada S-Function es un archivo MEX distinto y no comparte variables con los
 * demás, por lo que el estado viaja por la señal "llave" que sale de
 * daqPicInicio: la dirección del estado expresada como double. En Windows x64
 * las direcciones de usuario ocupan 47 bits y el double las representa sin
 * pérdida; daqPicInicio lo verifica al arrancar.
 */
#ifndef DAQ_BLOQUES_H
#define DAQ_BLOQUES_H

#include <stddef.h>
#include <stdint.h>
#include "../../protocolo/daq_protocolo.h"

#define DAQ_BLOQUES_MAGIA 0x44415150u   /* "DAQP" */

struct EstadoDaq {
    uint32_t magia;         /* DAQ_BLOQUES_MAGIA mientras el estado es válido */

    /* Escrito por daqPicEscribir en mdlOutputs */
    uint16_t dac;           /* código del DAC del paso actual */
    bool salida;            /* hay un bloque daqPicEscribir en el modelo */

    /* Escrito por daqPicInicio en mdlStart y mdlUpdate */
    double posicion;        /* cuentas del encoder, relativas al inicio */
    double errores;         /* errores de comunicación acumulados */
    double atrasos;         /* pasos atrasados respecto al tiempo real */
};

static inline double daq_llave(const EstadoDaq *e)
{
    return (double)(uintptr_t)e;
}

/* Devuelve el estado que indica la llave, o NULL si la señal no viene de
 * daqPicInicio. */
static inline EstadoDaq *daq_estado(double llave)
{
    if (!(llave > 0.0) || llave != (double)(uintptr_t)llave) {
        return NULL;
    }
    EstadoDaq *e = (EstadoDaq *)(uintptr_t)llave;
    return e->magia == DAQ_BLOQUES_MAGIA ? e : NULL;
}

#endif

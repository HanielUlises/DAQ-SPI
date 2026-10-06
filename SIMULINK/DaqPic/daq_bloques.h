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

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

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

/* Indica si se pueden leer n bytes a partir de p. En Windows lo consulta con
 * VirtualQuery; fuera de Windows (el arnés de herramientas/arnes_simulink) se
 * confía en las comprobaciones de daq_estado. */
static inline bool daq_legible(const void *p, size_t n)
{
#ifdef _WIN32
    MEMORY_BASIC_INFORMATION mbi;
    const DWORD lectura = PAGE_READONLY | PAGE_READWRITE | PAGE_WRITECOPY |
                          PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE |
                          PAGE_EXECUTE_WRITECOPY;
    if (VirtualQuery(p, &mbi, sizeof mbi) == 0 || mbi.State != MEM_COMMIT ||
        !(mbi.Protect & lectura) || (mbi.Protect & PAGE_GUARD)) {
        return false;
    }
    return (const char *)p + n <= (const char *)mbi.BaseAddress + mbi.RegionSize;
#else
    (void)p;
    (void)n;
    return true;
#endif
}

/* Devuelve el estado que indica la llave, o NULL si la señal no viene de
 * daqPicInicio. Una llave equivocada (una constante, otra señal) no se lee:
 * debe ser un entero de 47 bits, alineado, fuera de los primeros 64 KB (que
 * Windows nunca asigna) y apuntar a memoria legible con la marca. */
static inline EstadoDaq *daq_estado(double llave)
{
    if (!(llave >= 65536.0 && llave < 140737488355328.0) ||   /* [2^16, 2^47) */
        llave != (double)(uintptr_t)llave) {
        return NULL;
    }
    const uintptr_t dir = (uintptr_t)llave;
    if (dir % alignof(EstadoDaq) != 0 || !daq_legible((const void *)dir, sizeof(EstadoDaq))) {
        return NULL;
    }
    EstadoDaq *e = (EstadoDaq *)dir;
    return e->magia == DAQ_BLOQUES_MAGIA ? e : NULL;
}

#endif

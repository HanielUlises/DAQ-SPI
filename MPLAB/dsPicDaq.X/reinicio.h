/*
 * Causa del último reinicio del dsPIC, para reportarla a la PC en el byte
 * status (DAQ_STATUS_REINICIO).
 */
#ifndef REINICIO_H
#define REINICIO_H

#include <stdint.h>

/* Lee y limpia RCON. Debe llamarse al inicio de main, antes de cualquier
 * otra inicialización. */
void reinicio_registrar(void);

/* Causa registrada (DAQ_REINICIO_*), 0 si no se pudo determinar. */
uint8_t reinicio_causa(void);

#endif /* REINICIO_H */

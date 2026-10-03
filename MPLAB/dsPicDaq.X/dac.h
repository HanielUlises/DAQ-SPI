/*
 * DAC8554 (Texas Instruments) en SPI2. Se utiliza únicamente el canal A.
 */
#ifndef DAC_H
#define DAC_H

#include <stdint.h>

void dac_inicializar(void);
void dac_escribir(uint16_t valor);

#endif /* DAC_H */

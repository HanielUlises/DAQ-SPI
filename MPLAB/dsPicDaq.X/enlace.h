/*
 * Enlace SPI con la PC: SPI1 en modo esclavo atendido por DMA, con tramas de
 * longitud fija delimitadas por CS (RB10).
 *
 * Recursos que ocupa: SPI1, canales DMA0 (RX) y DMA1 (TX), INT1 (flanco de
 * subida de CS) y Timer1 (temporizador de vigilancia).
 */
#ifndef ENLACE_H
#define ENLACE_H

#include <stdbool.h>
#include <stdint.h>

void enlace_inicializar(void);

/* Devuelve true si hay un valor nuevo para el DAC y lo copia en *valor. */
bool enlace_dac_pendiente(uint16_t *valor);

#endif /* ENLACE_H */

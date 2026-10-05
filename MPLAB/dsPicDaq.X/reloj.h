/*
 * Reloj del sistema: FRC de 8 MHz con el PLL primario.
 *
 * FVCO = 8 MHz * 125 / 1 = 1000 MHz, FPLLO = FVCO / (5 * 1) = 200 MHz,
 * FOSC = FPLLO / 2 = 100 MHz y FCY = FP = FOSC / 2 = 50 MHz
 * (DS70005399C, ejemplo 9-2).
 *
 * MCC configura el FRC sin PLL; si se regenera el código, CLOCK_Initialize
 * deja el FRC y reloj_inicializar cambia al PLL después. Requiere el cambio
 * de reloj habilitado (FCKSM = CSECMD en config_bits.c).
 */
#ifndef RELOJ_H
#define RELOJ_H

#define RELOJ_FCY   50000000UL  /* también es el reloj de periféricos (FP) */

void reloj_inicializar(void);

#endif /* RELOJ_H */

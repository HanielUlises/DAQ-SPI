#include <libpic30.h>

#include "dac.h"
#include "mcc_generated_files/spi_host/spi2.h"
#include "mcc_generated_files/system/pins.h"

/* Byte de control: A1 = A0 = 0, LD1:LD0 = 01 (actualización de un solo
 * canal), DAC Sel = 00 (canal A), PD0 = 0. */
#define DAC8554_CTRL_CANAL_A    0x10u

/* Un periodo de SCK en ciclos de instrucción (SCK = FP / 32, FCY = FP) */
#define DAC_TCY_POR_SCK         32u

void dac_inicializar(void)
{
    IO_SYNC_SetHigh();
    SPI2_Open(0);

    /* El DAC8554 toma DIN en el flanco de bajada de SCLK. MCC deja SPI2 en
     * modo 0 (CKE = 1), en el que SDO2 cambia justo en ese flanco; con
     * CKE = 0 (modo 1) cambia en el de subida y el dato está estable cuando
     * el DAC lo lee. CKE solo puede cambiarse con el módulo apagado. */
    SPI2CON1Lbits.SPIEN = 0U;
    SPI2CON1Lbits.CKE = 0U;
    SPI2CON1Lbits.SPIEN = 1U;
}

/* SPI2 usa el divisor de MCC (SPI2BRGL = 15): SCK = FP / 32 = 1.56 MHz con
 * FP = 50 MHz, muy por debajo del máximo del DAC8554. */

/* Trama de 24 bits enmarcada por SYNC. SPI2_ByteExchange regresa al recibir
 * el ultimo bit, que puede llegar antes del flanco de bajada 24 de SCLK; si
 * SYNC sube antes de ese flanco, el DAC8554 descarta la trama. Por eso se
 * espera un periodo de SCK antes de subir SYNC. */
void dac_escribir(uint16_t valor)
{
    IO_SYNC_SetLow();
    (void)SPI2_ByteExchange(DAC8554_CTRL_CANAL_A);
    (void)SPI2_ByteExchange((uint8_t)(valor >> 8));
    (void)SPI2_ByteExchange((uint8_t)valor);
    __delay32(DAC_TCY_POR_SCK);
    IO_SYNC_SetHigh();
}

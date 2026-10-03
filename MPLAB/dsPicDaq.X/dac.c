#include "dac.h"
#include "mcc_generated_files/spi_host/spi2.h"
#include "mcc_generated_files/system/pins.h"

/* Byte de control: A1 = A0 = 0, LD1:LD0 = 01 (actualización de un solo
 * canal), DAC Sel = 00 (canal A), PD0 = 0. */
#define DAC8554_CTRL_CANAL_A    0x10u

void dac_inicializar(void)
{
    IO_SYNC_SetHigh();
    SPI2_Open(0);
}

/* Trama de 24 bits enmarcada por SYNC. SPI2_ByteExchange espera a que cada
 * byte termine, por lo que SYNC sube después del último bit. */
void dac_escribir(uint16_t valor)
{
    IO_SYNC_SetLow();
    (void)SPI2_ByteExchange(DAC8554_CTRL_CANAL_A);
    (void)SPI2_ByteExchange((uint8_t)(valor >> 8));
    (void)SPI2_ByteExchange((uint8_t)valor);
    IO_SYNC_SetHigh();
}

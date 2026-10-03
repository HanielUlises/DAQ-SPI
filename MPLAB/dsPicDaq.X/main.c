/*
 * DAQ-SPI: firmware con trama full-duplex de longitud fija.
 *
 * El enlace con la PC (SPI1 + DMA) se atiende por completo en interrupciones;
 * el lazo principal solo transmite al DAC los valores nuevos.
 */
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/qei/qei1.h"
#include "../../protocolo/daq_protocolo.h"
#include "dac.h"
#include "enlace.h"

int main(void)
{
    uint16_t valor;
    uint16_t ultimo;

    SYSTEM_Initialize();
    QEI1_Enable();

    dac_inicializar();
    dac_escribir(DAQ_DAC_CERO);
    ultimo = DAQ_DAC_CERO;

    enlace_inicializar();

    while (1)
    {
        if (enlace_dac_pendiente(&valor) && (valor != ultimo))
        {
            dac_escribir(valor);
            ultimo = valor;
        }
    }
}

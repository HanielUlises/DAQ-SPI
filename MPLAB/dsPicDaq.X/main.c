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
#include "reinicio.h"
#include "reloj.h"

int main(void)
{
    uint16_t valor;
    uint16_t ultimo;

    reinicio_registrar();
    SYSTEM_Initialize();
    reloj_inicializar();

    /* Filtro digital del QEI: MCC lo deja con el reloj de periféricos sin
     * dividir; con FP = 50 MHz se divide entre 16 para rechazar pulsos de
     * menos de ~1 us, como con el reloj anterior de 4 MHz. */
    QEI1IOCbits.QFDIV = 4;
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

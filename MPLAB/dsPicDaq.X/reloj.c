#include <xc.h>
#include "reloj.h"

void reloj_inicializar(void)
{
    /* Los divisores del PLL sólo se modifican mientras se opera sin PLL */
    CLKDIVbits.PLLPRE = 1;          /* N1 = 1 */
    PLLFBDbits.PLLFBDIV = 125;      /* M = 125 */
    PLLDIVbits.POST1DIV = 5;        /* N2 = 5 */
    PLLDIVbits.POST2DIV = 1;        /* N3 = 1 */

    /* Cambio a FRC con PLL (NOSC = 0b001) */
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    while (OSCCONbits.OSWEN != 0) {
    }
    while (OSCCONbits.LOCK != 1) {
    }
}

// --- Escritura de DAC desde simulink ---
//Pines
//     SPI1 (Client) -> SCK-B13; SDI-B14; SDO-B11; SS-B10;
//          FTD      -> AD0;     AD1;     AD2;     AD3;  

//     SPI2 (Host)   -> SCK-B0;  SDO-B2;  SYNC-B4;
//          DAC      -> 10;      11;      9;

#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/spi_client/spi1.h"
#include "mcc_generated_files/spi_host/spi2.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/uart/uart1.h"
#define FCY 4000000UL
#include <stdio.h>       
#include <libpic30.h> 
#include <xc.h>
/*
    Main application
*/

int main(void)
{
    SYSTEM_Initialize();
    SPI1_Open(0);
    SPI2_Open(0);
    IO_LED_SetHigh();
    uint8_t byteLeido = 0;
    uint8_t estado = 0;
    uint8_t parteAlta = 0;
    uint8_t parteBaja = 0;
    uint16_t valorDac = 0;
    
    while(1)
    {   
        IO_SYNC_SetHigh();
        if(SPI1_IsRxReady())
        {
            //IO_LED_SetLow();
            
            byteLeido = SPI1_ByteRead(); 
            
            switch(estado)
            {
                case 0: 
                    
                    if (byteLeido == 0xAA)
                    {
                        estado = 1;
                        IO_LED_SetLow();
                    }
                    break;
                case 1:
                    
                        parteAlta = byteLeido;
                        estado = 2;  
                        printf("%d\n",parteAlta);
                    break;
                case 2:
                    
                    parteBaja = byteLeido;
                    printf("%d\n",parteBaja);
                    estado = 0;
                    
                    IO_SYNC_SetLow();
       
                    SPI2_ByteExchange(0x10);
                    SPI2_ByteExchange(parteAlta);
                    SPI2_ByteExchange(parteBaja);        

                    IO_SYNC_SetHigh();
                    IO_LED_SetHigh();
                    break;  
                    
            }
        }
            
    }    
}
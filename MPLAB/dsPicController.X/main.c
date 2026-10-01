#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/spi_host/spi2.h"
#include "mcc_generated_files/spi_client/spi1.h"
#include "mcc_generated_files/qei/qei1.h"
#define FCY 8000000UL
#include <stdio.h>       
#include <libpic30.h> 
#include <xc.h>

void lecturaEncoder();
void escrituraDac();

//Encoder
uint8_t bandera = 0; 
uint8_t Tx = 0;

typedef union{
    int32_t posAct;
    uint8_t bytes[4];
} pulsos8b;

pulsos8b datosEncoder;

//DAC
uint8_t byteLeido = 0;
uint8_t estado = 0;
uint8_t parteAlta = 0;
uint8_t parteBaja = 0;
uint16_t valorDac = 0;



int main(void)
{
    SYSTEM_Initialize();
    QEI1_Enable();
    SPI1_Open(0);
    SPI2_Open(0);

    while(1)
    {
      escrituraDac();
      lecturaEncoder();
    }    
}

void lecturaEncoder()
{
    datosEncoder.posAct = QEI1_PositionCountRead();
        
        IO_LED_Toggle(); 
        
        if(SPI1_IsTxReady())
        {
            SPI1_ByteWrite(0xAA);
            SPI1_ByteWrite(0x55);

            SPI1_ByteWrite(datosEncoder.bytes[0]);
            SPI1_ByteWrite(datosEncoder.bytes[1]);
            SPI1_ByteWrite(datosEncoder.bytes[2]);
            SPI1_ByteWrite(datosEncoder.bytes[3]);
        }
}
void escrituraDac()
{
    IO_SYNC_SetHigh();
        if(SPI1_IsRxReady())
        {
            byteLeido = SPI1_ByteRead(); 
            IO_LED_SetLow();
            switch(estado)
            {
                case 0: 
                    
                    if (byteLeido == 0xAA)
                    {
                        estado = 1;
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
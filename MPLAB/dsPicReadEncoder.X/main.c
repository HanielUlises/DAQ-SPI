//----- LECTURA DE ENCODER -----
//Pines:  
//     SPI -> SCK-B13; SDI-B14; SDO-B11; SS-B10;
//     QEI -> A-B8; B-B9;

#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/qei/qei1.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/spi_client/spi1.h"
#define FCY 8000000UL
#include <stdio.h>       
#include <libpic30.h> 
#include <xc.h>

uint8_t bandera = 0; 
uint8_t Tx = 0;

typedef union{
    int32_t posAct;
    uint8_t bytes[4];
} pulsos8b;

pulsos8b datosEncoder;

int main(void)
{   
    SYSTEM_Initialize();
    _SWDTEN = 0; // Apaga el watchdog por software
    QEI1_Enable();
    SPI1_Client_Open(0);
    
    while(1)
    {    
        datosEncoder.posAct = QEI1_PositionCountRead();
        
        //datosEncoder.posAct = (uint32_t)POS1CNTH<<16 | (uint32_t)POS1CNTL;
        
        IO_LED_Toggle(); 
        if(SPI1_Client_IsTxReady)
        {
            
            SPI1_Client_ByteWrite(0xAA);
            SPI1_Client_ByteWrite(0x55);
            //_LATD10 = 0;
            SPI1_Client_ByteWrite(datosEncoder.bytes[0]);
            SPI1_Client_ByteWrite(datosEncoder.bytes[1]);
            SPI1_Client_ByteWrite(datosEncoder.bytes[2]);
            SPI1_Client_ByteWrite(datosEncoder.bytes[3]);
          
        }
        printf("%ld\n",datosEncoder.posAct);  
    }    
}
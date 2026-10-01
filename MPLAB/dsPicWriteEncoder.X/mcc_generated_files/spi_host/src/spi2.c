
/**
 * SPI2 Generated Driver Source File
 * 
 * @file        spi2.c
 * 
 * @ingroup     spihostdriver
 * 
 * @brief       This is the generated driver source file for SPI2 driver.
 *
 * @skipline @version     Firmware Driver Version 1.1.3
 *
 * @skipline @version     PLIB Version 1.4.3
 *
 * @skipline    Device : dsPIC33CK64MC105
*/

/*
© [2026] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

// Section: Included Files
#include <xc.h>
#include "../spi2.h"

// Section: File specific data type definitions

/** 
 @ingroup spidriver
  @brief Dummy data to be sent for half duplex communication.
*/
#define SPI2_DUMMY_DATA 0x0

//Defines an object for SPI_HOST_INTERFACE.

const struct SPI_HOST_INTERFACE SPI2_Host = {
    .Initialize         = &SPI2_Initialize,
    .Deinitialize       = &SPI2_Deinitialize,
    .Close              = &SPI2_Close,
    .Open               = &SPI2_Open,
    .BufferExchange     = &SPI2_BufferExchange,
    .BufferRead         = &SPI2_BufferRead,
    .BufferWrite        = &SPI2_BufferWrite,
    .ByteExchange       = &SPI2_ByteExchange,
    .ByteRead           = &SPI2_ByteRead,    
    .ByteWrite          = &SPI2_ByteWrite,
    .IsRxReady          = &SPI2_IsRxReady,
    .IsTxReady          = &SPI2_IsTxReady,
};
        

/**
 @ingroup spihostdriver
 @struct SPI2_CONFIG 
 @brief Defines the SPI2 configuration.
*/
struct SPI2_HOST_CONFIG
{ 
    uint16_t controlRegister1; //SPI2BRGL
    uint16_t controlRegister2; //SPI2CON1L
};

static const struct SPI2_HOST_CONFIG config[] = {  
                                        { 
                                            /*Configuration setting for HOST_CONFIG.
                                            SPI Mode : Mode 1, Sampled at : Middle, Data Width : 8 bits, Clock Frequency : 125 kHz*/
                                            0xfU,//SPI2BRGL
                                            0x21U,//SPI2CON1L
                                        },
                                    };

// Section: Driver Interface Function Definitions

void SPI2_Initialize (void)
{
    // SPIBRGL 15; 
    SPI2BRGL = 0xFU;
    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; AUDMONO stereo; FRMCNT 0x0; MSSEN disabled; FRMPOL disabled; IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
    SPI2CON1H = 0x0U;
    // WLENGTH 0; 
    SPI2CON2L = 0x0U;
    // SPIROV disabled; FRMERR disabled; 
    SPI2STATL = 0x0U;
    // SPIURDTL 0; 
    SPI2URDTL = 0x0U;
    // SPIURDTH 0; 
    SPI2URDTH = 0x0U;
    // SPIEN disabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; SSEN disabled; MSTEN Host; MODE16 disabled; SMP Middle; DISSCK disabled; SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; SPISIDL disabled; ENHBUF enabled; DISSDI disabled; 
    SPI2CON1L = 0x21U;
}

void SPI2_Deinitialize (void)
{
    SPI2_Close();
    
    SPI2BRGL = 0x0U;
    SPI2CON1L = 0x0U;
    SPI2CON1H = 0x0U;
    SPI2CON2L = 0x0U;
    SPI2STATL = 0x28U;
    SPI2URDTL = 0x0U;
    SPI2URDTH = 0x0U;
}

void SPI2_Close(void)
{
    SPI2CON1Lbits.SPIEN = 0U;
}

bool SPI2_Open(uint8_t spiConfigIndex)
{
    bool status = false;
    if(!SPI2CON1Lbits.SPIEN)
    {
        SPI2BRGL = config[spiConfigIndex].controlRegister1;
        SPI2CON1L = config[spiConfigIndex].controlRegister2;
        SPI2CON1Lbits.SPIEN = 1U;
        
        status = true;
    }
    return status;
}

uint8_t SPI2_ByteExchange(uint8_t byteData)
{
    while(1U == SPI2STATLbits.SPITBF)
    {

    }

    SPI2BUFL = byteData;

    while (1U == SPI2STATLbits.SPIRBE)
    {
    
    }

    return SPI2BUFL;
}

void SPI2_ByteWrite(uint8_t byteData)
{
    while(1U == SPI2STATLbits.SPITBF)
    {

    }
    
    SPI2BUFL = byteData;
}

uint8_t SPI2_ByteRead(void)
{
    while (1U == SPI2STATLbits.SPIRBE)
    {
    
    }
    
    return SPI2BUFL;
}

void SPI2_BufferExchange(void *bufferData, size_t bufferSize)
{
    uint16_t dataSentCount = 0U;
    uint16_t dataReceivedCount = 0U;

    while(1U == SPI2STATLbits.SPITBF)
    {

    }

    if (SPI2CON1Lbits.MODE32 == 1U)
    {
        // ---------------- 32-bit Mode  ----------------
        uint16_t *data = (uint16_t *)bufferData;
        uint16_t sendAddressIncrement = 0U;
        uint16_t receiveAddressIncrement = 0U;

        bufferSize >>= 2U;

        while (dataSentCount < bufferSize)
        {
            if ( 1U != SPI2STATLbits.SPITBF )
            {
                SPI2BUFL = data[sendAddressIncrement];
                SPI2BUFH = data[sendAddressIncrement + 1U];
                sendAddressIncrement += 2U;
                dataSentCount++;
            }

            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[receiveAddressIncrement] = SPI2BUFL;
                data[receiveAddressIncrement + 1U] = SPI2BUFH;
                receiveAddressIncrement += 2U;
                dataReceivedCount++;
            }
        }
        while (dataReceivedCount < bufferSize)
        {
            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[receiveAddressIncrement] = SPI2BUFL;
                data[receiveAddressIncrement + 1U] = SPI2BUFH;
                receiveAddressIncrement += 2U;
                dataReceivedCount++;
            }
        }
    }
    else if(SPI2CON1Lbits.MODE16 == 1U)
    {
            // ---------------- 16-bit Mode --------------------
            uint16_t *data = (uint16_t *)bufferData;
            bufferSize >>= 1U;

            while (dataSentCount < bufferSize)
            {
                if ( 1U != SPI2STATLbits.SPITBF )
                {
                    SPI2BUFL = data[dataSentCount];
                    dataSentCount++;
                }

                if (0U == SPI2STATLbits.SPIRBE)
                {
                    data[dataReceivedCount] = SPI2BUFL;
                    dataReceivedCount++;
                }
            }
            while (dataReceivedCount < bufferSize)
            {
                if (0U == SPI2STATLbits.SPIRBE)
                {
                    data[dataReceivedCount] = SPI2BUFL;
                    dataReceivedCount++;
                }
            }
    }
    else
    {
        // ---------------- 8-bit Mode --------------------
        uint8_t *data = (uint8_t *)bufferData;

        while (dataSentCount < bufferSize)
        {
            if ( 1U != SPI2STATLbits.SPITBF )
            {
                SPI2BUFL = data[dataSentCount];
                dataSentCount++;
            }

            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[dataReceivedCount] = SPI2BUFL;
                dataReceivedCount++;
            }
        }
        while (dataReceivedCount < bufferSize)
        {
            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[dataReceivedCount] = SPI2BUFL;
                dataReceivedCount++;
            }
        }
    }
}

void SPI2_BufferWrite(void *bufferData, size_t bufferSize)
{   
    uint16_t dataSentCount = 0U;
    uint16_t dataReceivedCount = 0U;

    while(1U == SPI2STATLbits.SPITBF)
    {

    }   
    // ---------------- 32-bit Mode  ----------------
    if (SPI2CON1Lbits.MODE32 == 1U)
    {
        uint16_t *data = (uint16_t *)bufferData; 
        uint16_t addressIncrement = 0U;

        bufferSize >>= 2U;

        while (dataSentCount < bufferSize)
        {
            if ( 1U != SPI2STATLbits.SPITBF )
            {
                SPI2BUFL = data[addressIncrement];
                SPI2BUFH = data[addressIncrement + 1U];
                addressIncrement += 2U;
                dataSentCount++;
            }

            if (0U == SPI2STATLbits.SPIRBE)
            {
                (void)SPI2BUFL; //Dummy Read
                (void)SPI2BUFH;
                dataReceivedCount++;
            }

        }
        while (dataReceivedCount < bufferSize)
        {
            if (0U == SPI2STATLbits.SPIRBE)
            {
                (void)SPI2BUFL; //Dummy Read
                (void)SPI2BUFH;
                dataReceivedCount++;
            }
        }  
    } 
    else if(SPI2CON1Lbits.MODE16 == 1U)
    {
        // ---------------- 16-bit Mode --------------------
        uint16_t *data = (uint16_t *)bufferData;    
        bufferSize >>= 1;

        while (dataSentCount < bufferSize)
        {
            if ( 1U != SPI2STATLbits.SPITBF )
            {
                SPI2BUFL = data[dataSentCount];
                dataSentCount++;
            }

            if (0U == SPI2STATLbits.SPIRBE)
            {
                (void)SPI2BUFL; //Dummy Read
                dataReceivedCount++;
            }
        }
        while (dataReceivedCount < bufferSize)
        {
            if (0U == SPI2STATLbits.SPIRBE)
            {
                (void)SPI2BUFL; //Dummy Read
                dataReceivedCount++;
            }
        }  
    }
    else
    {
        // ---------------- 8-bit Mode --------------------
        uint8_t *data = (uint8_t *)bufferData;

        while (dataSentCount < bufferSize)
        {
            if ( 1U != SPI2STATLbits.SPITBF )
            {
                SPI2BUFL = data[dataSentCount];
                dataSentCount++;
            }

            if (0U == SPI2STATLbits.SPIRBE)
            {
                (void)SPI2BUFL; //Dummy Read
                dataReceivedCount++;
            }
        }
        while (dataReceivedCount < bufferSize)
        {
            if (0U == SPI2STATLbits.SPIRBE)
            {
                (void)SPI2BUFL; //Dummy Read
                dataReceivedCount++;
            }
        }  
    }
}

void SPI2_BufferRead(void *bufferData, size_t bufferSize)
{   
    uint16_t dataSentCount = 0U;
    uint16_t dataReceivedCount = 0U;
    
    while(1U == SPI2STATLbits.SPITBF)
    {

    }

    // ---------------- 32-bit Mode  ----------------
    if (SPI2CON1Lbits.MODE32 == 1U)
    {
        uint16_t *data = (uint16_t *)bufferData;
        const uint16_t wData = SPI2_DUMMY_DATA;
        uint16_t addressIncrement = 0U;

        bufferSize >>= 2U;

        while (dataSentCount < bufferSize)
        {
            if ( 1U != SPI2STATLbits.SPITBF )
            {
                SPI2BUFL = wData;
                SPI2BUFH = wData;
                dataSentCount++;
            }

            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[addressIncrement] = SPI2BUFL;
                data[addressIncrement + 1U] = SPI2BUFH;
                addressIncrement += 2U;
                dataReceivedCount++;
            }
        }
        while (dataReceivedCount < bufferSize)
        {
            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[addressIncrement] = SPI2BUFL;
                data[addressIncrement + 1U] = SPI2BUFH;
                addressIncrement += 2U;
                dataReceivedCount++;
            }
        }
    }
    else if(SPI2CON1Lbits.MODE16 == 1U)
    {
        // ---------------- 16-bit Mode --------------------
        uint16_t *data = (uint16_t *)bufferData; 
        const uint16_t wData = SPI2_DUMMY_DATA;  
        bufferSize >>= 1;

        while (dataSentCount < bufferSize)
        {
            if ( 1U != SPI2STATLbits.SPITBF )
            {
                SPI2BUFL = wData;
                dataSentCount++;
            }

            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[dataReceivedCount] = SPI2BUFL;
                dataReceivedCount++;
            }
        }
        while (dataReceivedCount < bufferSize)
        {
            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[dataReceivedCount] = SPI2BUFL;
                dataReceivedCount++;
            }
        } 
    }
    else
    {
        // ---------------- 8-bit Mode --------------------
        uint8_t *data = (uint8_t *)bufferData;
        const uint8_t wData = SPI2_DUMMY_DATA;

        while (dataSentCount < bufferSize)
        {
            if ( 1U != SPI2STATLbits.SPITBF )
            {
                SPI2BUFL = wData;
                dataSentCount++;
            }

            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[dataReceivedCount] = SPI2BUFL;
                dataReceivedCount++;
            }
        }
        while (dataReceivedCount < bufferSize)
        {
            if (0U == SPI2STATLbits.SPIRBE)
            {
                data[dataReceivedCount] = SPI2BUFL;
                dataReceivedCount++;
            }
        } 
    }
}

bool SPI2_IsRxReady(void)
{    
    return (!SPI2STATLbits.SPIRBE);
}

bool SPI2_IsTxReady(void)
{    
    return (!SPI2STATLbits.SPITBF);
}
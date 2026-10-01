
/**
 * SPI2 Generated Driver Header File
 * 
 * @file        spi2.h
 * 
 * @ingroup     spihostdriver
 * 
 * @brief       This is the generated driver header file for the SPI2 driver
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

#ifndef SPI2_H
#define SPI2_H

// Section: Included Files
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "spi_host_interface.h"

// Section: Data Type Definitions

/**
 @ingroup   spihostdriver
 @brief     Structure object of type SPI_HOST_INTERFACE assigned with name 
            displayed in the Melody Driver User interface. A structure pointer can be used to achieve portability
            across the SPI HOST having same interface structure.
*/

extern const struct SPI_HOST_INTERFACE SPI2_Host;
      
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_Initialize API
 */
#define SPI2_Host_Initialize SPI2_Initialize
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_Deinitialize API
 */
#define SPI2_Host_Deinitialize SPI2_Deinitialize
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_Close API
 */
#define SPI2_Host_Close SPI2_Close
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_Open API
 */
#define SPI2_Host_Open SPI2_Open
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_ByteExchange API
 */
#define SPI2_Host_ByteExchange SPI2_ByteExchange
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_BufferExchange API
 */
#define SPI2_Host_BufferExchange SPI2_BufferExchange
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_BufferWrite API
 */
#define SPI2_Host_BufferWrite SPI2_BufferWrite
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_BufferRead API
 */
#define SPI2_Host_BufferRead SPI2_BufferRead
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_ByteWrite API
 */
#define SPI2_Host_ByteWrite SPI2_ByteWrite
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_ByteRead API
 */
#define SPI2_Host_ByteRead SPI2_ByteRead
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_IsRxReady API
 */
#define SPI2_Host_IsRxReady SPI2_IsRxReady
/**
 * @ingroup  spidriver
 * @brief    This macro defines the Custom Name for \ref SPI2_IsTxReady API
 */
#define SPI2_Host_IsTxReady SPI2_IsTxReady

/**
@ingroup spihostdriver
@enum SPI2_HOST_USER_CONFIG
@brief Enum list is the Custom names for the SPI2 HOST multiple configuration, 
       configured by user in MCC Melody user interface. 
       These Host Configurations can be used to switch at runtime to interact with multiple clients.
@note The enum list in the Help document might be just a reference to illustrate multiple configuration.
      Generated enum list is based on the configuration done by user in the MCC Melody user interface
*/

enum SPI2_HOST_USER_CONFIG{
     /**Custom name for configuration setting:
    SPI Mode : Mode 0, Sampled at : Middle, Data Width : 8 bits, Clock Frequency : 125 kHz**/
    HOST_CONFIG,
    
};

// Section: Driver Interface Function


/**
 * @ingroup spihostdriver
 * @brief   Initializes SPI2 module, using the given initialization data. 
 *          This function must be called before any other SPI2 function is called.
 * @param   none
 * @return  none  
 */
void SPI2_Initialize (void);


/**
 * @ingroup spihostdriver
 * @brief   Deinitializes the SPI2 to POR values.
 * @param   none
 * @return  none  
 */
void SPI2_Deinitialize(void);


/**
 * @ingroup spihostdriver
 * @brief   Disables the SPI2 module. 
 * @pre     The \ref SPI2_Open function must be called before calling this function.
 * @param   none
 * @return  none  
 */
void SPI2_Close (void);


/**
 * @ingroup     spihostdriver
 * @brief       Configures SPI2 module with user defined unique configuration. 
 * @param[in]   spiConfigIndex    - \ref SPI2_USER_CONFIG Enum value 
 * @return bool true   -   SPI2 is configured successfully.
 * @return bool false  -   SPI2 is not configured successfully.  
 */
bool SPI2_Open(uint8_t spiConfigIndex);

/**
 * @ingroup     spihostdriver
 * @brief       Exchanges one byte of data. This is a blocking function.
 * @pre         The \ref SPI2_IsTxReady function must be called before calling this function.
 * @param[in]   data- Data byte to be exchanged.  
 * @return      none  
 * @note        This API must be used only for SPI transfer with 8-bit data width.
 */
uint8_t SPI2_ByteExchange(uint8_t byteData);

/**
 * @ingroup         spihostdriver
 * @brief           This function is used to transmit data from the buffer pointer 
 *                  and updates the received data in the same buffer location.
 *                  This is a blocking function.
 *
 *                  This API supports SPI transfers with data widths of 8-bit, 16-bit,
 *                  or 32-bit. The transfer width is defined by the user defined unique Configurations
 *                  in MCC user interface.
 * @pre             The \ref SPI2_IsTxReady function must be called before calling this function.
 * @param[in,out]   bufferData     - Address of data buffer to be excanged.
 * @param[in]       bufferSize     - Number of bytes in a buffer.  
 * @return          none  
 */
void SPI2_BufferExchange(void *bufferData, size_t bufferSize);

/**
 * @ingroup     spihostdriver
 * @brief       Writes data from a buffer. This is a blocking routine.
 *
 *              This API supports SPI transfers with data widths of 8-bit, 16-bit,
 *              or 32-bit. The transfer width is defined by the user defined unique Configurations
 *              in MCC user interface.
 * @pre         The \ref SPI2_IsTxReady function must be called before calling this function.
 * @param[in]   bufferData     - Address of data buffer to be written.
 * @param[in]   bufferSize     - Number of bytes in a buffer.  
 * @return      none  
 */
void SPI2_BufferWrite(void *bufferData, size_t bufferSize);

/**
 * @ingroup     spihostdriver
 * @brief       Reads data from a buffer. This is a blocking routine.
 *
 *              This API supports SPI transfers with data widths of 8-bit, 16-bit,
 *              or 32-bit. The transfer width is defined by the user defined unique Configurations
 *              in MCC user interface.
 * @pre         The \ref SPI2_IsRxReady function must be called before calling this function.
 * @param[out]  bufferData     - Address of data buffer to be read.
 * @param[in]   bufferSize     - Number of bytes in a buffer.  
 * @return      none  
 */
void SPI2_BufferRead(void *bufferData, size_t bufferSize);

/**
 * @ingroup     spihostdriver
 * @brief       Writes one byte of data.
 * @pre         The \ref SPI2_IsTxReady function must be called before calling this function.
 *              Also note that every byte write has to be followed by a byte read. 
 * @param[in]   byteData         - Data to be written.  
 * @return      none  
 * @note        This API must be used only for SPI transfer with 8-bit data width.
 */
void SPI2_ByteWrite(uint8_t byteData);

/**
 * @ingroup     spihostdriver
 * @brief       Reads one byte of data from SPI2
 * @pre         The \ref SPI2_IsRxReady function must be called before calling this function.
 *              Also note that byte read must be after byte write sequence.
 * @param       none
 * @return      Data read from SPI2.  
 * @note        This API must be used only for SPI transfer with 8-bit data width.
 */
uint8_t SPI2_ByteRead(void);

/**
 * @ingroup spihostdriver
 * @brief   Returns a boolean value if data is available to read.
 * @pre     The \ref SPI2_Open function must be called before calling this function.
 * @param   none
 * @return  true    - Data available to read
 * @return  false   - Data not available to read  
 */
bool SPI2_IsRxReady(void);


/**
 * @ingroup spihostdriver
 * @brief   Returns a boolean value if data can be written.
 * @pre     The \ref SPI2_Open function must be called before calling this function.
 * @param   none
 * @return  true    - Data can be written
 * @return  false   - Data buffer is full  
 */
bool SPI2_IsTxReady(void);

#endif //SPI2_H




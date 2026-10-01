
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define SIMPLIFIED_RTWTYPES_COMPATIBILITY
#include "rtwtypes.h"
#undef SIMPLIFIED_RTWTYPES_COMPATIBILITY
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define SIMPLIFIED_RTWTYPES_COMPATIBILITY
#include "rtwtypes.h"
#undef SIMPLIFIED_RTWTYPES_COMPATIBILITY
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <windows.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"

#pragma comment(lib, "libmpsse.lib")

#define APP_CHECK_STATUS(exp) {if (exp!=FT_OK){printf(" status(0x%x) != FT_OK\n", exp);}}

FT_HANDLE ftHandle;
int Started = 0;
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
 
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
extern void systemSPI_Start_wrapper(void);

void systemSPI_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
Started = 0;
    ChannelConfig channelConf;
    DWORD channels;
    FT_STATUS status;
    
    // Configuración 100kHz, Modo 0, CS en ADBUS3
    channelConf.ClockRate = 100000; 
    channelConf.LatencyTimer = 1;
    channelConf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;

    Init_libMPSSE();
    status = SPI_GetNumChannels(&channels);

    if (channels > 0) 
    {
        // Primer canal disponible (índice 0)
        status = SPI_OpenChannel(0, &ftHandle);
        if (status == FT_OK) {
            status = SPI_InitChannel(ftHandle, &channelConf);
            if (status == FT_OK) {
                Started = 1;
            }
        }
    }
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
extern void systemSPI_Outputs_wrapper(const real_T *u0,
			real_T *y0);

void systemSPI_Outputs_wrapper(const real_T *u0,
			real_T *y0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
if(Started == 1)
    {
        DWORD xfer = 0;
        uint8_t enviarPIC = (uint8_t)u0[0]; // Senoidal de Simulink
        uint8_t recibirPIC = 0;             // Respuesta

        // Intercambio simultáneo de 1 byte
        SPI_ReadWrite(ftHandle, &recibirPIC, &enviarPIC, 1, &xfer,
                      SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                      SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                      SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
        
        y0[0] = (real_T)recibirPIC; // Respuesta al Scope
    }
    else {
        y0[0] = 0;
    }
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Terminate function
 *
 */
extern void systemSPI_Terminate_wrapper(void);

void systemSPI_Terminate_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
if(Started == 1) {
        SPI_CloseChannel(ftHandle);
        Cleanup_libMPSSE();
    }
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}


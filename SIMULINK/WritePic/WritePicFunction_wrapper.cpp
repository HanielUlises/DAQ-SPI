
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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <windows.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"
#include "mex.h"

//#pragma comment(lib, "libmpsse.lib")

#define APP_CHECK_STATUS(exp) {if (exp!=FT_OK){printf(" status(0x%x) != FT_OK\n", exp);}else{;}};

FT_HANDLE ftHandle;
int Started = 0;

int primer_ciclo = 1;
int32_t offset_encoder = 0;
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
extern "C" void WritePicFunction_Start_wrapper(void);

void WritePicFunction_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
Started = 0;

primer_ciclo = 1;
offset_encoder = 0;

ChannelConfig channelConf;
DWORD channel;
DWORD channels;
FT_STATUS status;
    
    // Configuracin para leer del dsPIC
    channelConf.ClockRate = 1000000;
    channelConf.LatencyTimer = 1;
    channelConf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;

    Init_libMPSSE();
    status = SPI_GetNumChannels(&channels);

    if (channels > 0)
    {
        channel = 0; 
        status = SPI_OpenChannel(channel, &ftHandle);
        if (status == FT_OK)
        {
            status = SPI_InitChannel(ftHandle, &channelConf);
            if (status == FT_OK)
            {
                Started = 1; 
            }
            else 
            {
                printf("ERROR FTDI: Se encontro el chip pero fallo la inicializacion SPI.\n");
            }
        }
        else 
        {
            printf("ERROR FTDI: No se pudo abrir el puerto. Verifica que no este en uso.\n");
        }
    }
    else 
    {
        printf("ERROR FTDI: No se detecto ningun dispositivo USB FTDI conectado.\n");
    }
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
extern "C" void WritePicFunction_Outputs_wrapper(const real_T *u0,
			uint16_T *y0);

void WritePicFunction_Outputs_wrapper(const real_T *u0,
			uint16_T *y0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
FT_STATUS status;
    DWORD bytesTransferidos = 0;

    if (Started == 1)
    {
        
        double voltaje = u0[0];
        if (voltaje > 2.5) {
            voltaje = 2.5; // Lmite mximo
        }else if (voltaje < -2.5){
            voltaje = -2.5; //Limite minimo
        }

        //-2.5 = 0, 0 = 32767, 2.5 = 65535
        uint16_t valor_dac = (uint16_t)(13107.0 * (voltaje + 2.5));

        
        uint8_t bufferTx[3];
        bufferTx[0] = 0xAA; // Cabecera
        bufferTx[1] = (valor_dac >> 8) & 0xFF; // Parte Alta
        bufferTx[2] = valor_dac & 0xFF;        // Parte Baja
        
        status = SPI_Write(ftHandle, bufferTx, 3, &bytesTransferidos,
                               SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
                              
        y0[0] = (uint16_T)valor_dac; 
    }
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Terminate function
 *
 */
extern "C" void WritePicFunction_Terminate_wrapper(void);

void WritePicFunction_Terminate_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Terminate code goes here.
 */
    FT_STATUS status;
    DWORD bytesTransferidos1 = 0;
    uint8_t buffer[3];
    buffer[0] = 0xAA; // Cabecera
    buffer[1] = 0x00; // Parte Alta
    buffer[2] = 0x00;        // Parte Baja
    
    status = SPI_Write(ftHandle, buffer, 3, &bytesTransferidos1,
                           SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                           SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                           SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
    status = SPI_CloseChannel(ftHandle);
    APP_CHECK_STATUS(status);
    Cleanup_libMPSSE();
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}


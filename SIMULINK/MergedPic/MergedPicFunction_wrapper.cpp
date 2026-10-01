
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
/* Includes_BEGIN */
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
extern "C" void MergedPicFunction_Start_wrapper(void);

void MergedPicFunction_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
Started = 0;

primer_ciclo = 1;
offset_encoder = 0;

ChannelConfig channelConf;
DWORD channel;
DWORD channels;
FT_STATUS status;
    
    // Configuración para leer del dsPIC
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
extern "C" void MergedPicFunction_Outputs_wrapper(const real_T *u0);

void MergedPicFunction_Outputs_wrapper(const real_T *u0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
FT_STATUS status = FT_OK; 
DWORD bytesTransferidos = 0;
typedef union {
    int32_t valor_32;
    uint8_t bytes[4];
} Datos8_a_32;


if (Started == 1)
    {
    //WRITE PIC    
    double voltaje = u0[0];
        if (voltaje > 2.5) {
            voltaje = 2.5; // Límite máximo
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
    //READ PIC     
    /*UCHAR Buf[12]; 
        
        //Lectura de datos
        status = SPI_Read(ftHandle, Buf, 12, &bytesTransferidos,
                          SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                          SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                          SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
        
        //Lectura de datos guardados
        if (status == FT_OK && bytesTransferidos > 5)
        {
            for (int i = 0; i < (bytesTransferidos - 5); i++) 
            {
                // Busqueda de cabecera
                if (Buf[i] == 0xAA && Buf[i+1] == 0x55) 
                {
                    Datos8_a_32 rxData;
                    rxData.bytes[0] = Buf[i+2];
                    rxData.bytes[1] = Buf[i+3];
                    rxData.bytes[2] = Buf[i+4];
                    rxData.bytes[3] = Buf[i+5];
                    
                    //Reseteo a 0 para primera lectura
                    if (primer_ciclo == 1) 
                    {
                        offset_encoder = rxData.valor_32; 
                        primer_ciclo = 0;                 
                    }
                    
                    
                    int32_t valor_corregido = rxData.valor_32 - offset_encoder;
                    
                    y0[0] = (real_T)valor_corregido;                    
                    break; 
                }
            }
        }*/
    }
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Terminate function
 *
 */
extern "C" void MergedPicFunction_Terminate_wrapper(void);

void MergedPicFunction_Terminate_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
FT_STATUS status;

    status = SPI_CloseChannel(ftHandle);
    APP_CHECK_STATUS(status);
    Cleanup_libMPSSE();
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}


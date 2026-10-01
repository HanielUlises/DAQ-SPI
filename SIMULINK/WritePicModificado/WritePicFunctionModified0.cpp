#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  WritePicFunctionModified0

#include "simstruc.h"
#include <math.h>
#include <stdint.h>
#include <windows.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"

// Cambiamos printf por ssPrintf para que Simulink no colapse
#define APP_CHECK_STATUS(exp) {if (exp!=FT_OK){ssPrintf(" status(0x%x) != FT_OK\n", exp);}else{;}}

// Variables Globales (Standalone)
FT_HANDLE ftHandle;
int Started = 0;
int primer_ciclo = 1;
int32_t offset_encoder = 0;

/*====================*
 * S-function methods *
 *====================*/

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    // 1 Puerto de Entrada (Voltaje en double)
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    // 1 Puerto de Salida (y0 en uint16)
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_UINT16);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);

    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START 
static void mdlStart(SimStruct *S)
{
    Init_libMPSSE();
    
    DWORD channels = 0;
    FT_STATUS status = SPI_GetNumChannels(&channels);
    
    if (status == FT_OK && channels > 0) {
        if (SPI_OpenChannel(0, &ftHandle ) == FT_OK) {
            ChannelConfig channelConf;
            channelConf.ClockRate = 1000000; 
            channelConf.LatencyTimer = 1;
            channelConf.configOptions = SPI_CONFIG_OPTION_MODE0 | 
                                        SPI_CONFIG_OPTION_CS_DBUS3 | 
                                        SPI_CONFIG_OPTION_CS_ACTIVELOW;
            
            if (SPI_InitChannel(ftHandle, &channelConf) == FT_OK) {
                
            } else {
                mexErrMsgTxt("ERROR FTDI: Se encontro el chip pero fallo la inicializacion SPI.");
            }
        } else {
            mexErrMsgTxt("ERROR FTDI: No se pudo abrir el puerto. Verifica que no este en uso.");
        }
    } else {
        mexErrMsgTxt("ERROR FTDI: No se detecto ningun dispositivo USB FTDI conectado.");
    }
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Corrección: Usamos ssGetInputPortSignal en lugar de ssGetInputPortRealSignal
    const real_T *u0 = (const real_T *) ssGetInputPortSignal(S, 0);
    uint16_T *y0 = (uint16_T *) ssGetOutputPortSignal(S, 0);
    
    FT_STATUS status;
    DWORD bytesTransferidos = 0;

    if (Started == 1)
    {
        double voltaje = u0[0];
        if (voltaje > 2.5) {
            voltaje = 2.5; 
        } else if (voltaje < -2.5) {
            voltaje = -2.5; 
        }

        uint16_t valor_dac = (uint16_t)(13107.0 * (voltaje + 2.5));

        uint8_t bufferTx[3];
        bufferTx[0] = 0xAA; 
        bufferTx[1] = (valor_dac >> 8) & 0xFF; 
        bufferTx[2] = valor_dac & 0xFF;        
        
        status = SPI_Write(ftHandle, bufferTx, 3, &bytesTransferidos,
                               SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
                              
        y0[0] = (uint16_T)valor_dac; 
    }
}

static void mdlTerminate(SimStruct *S)
{
    // Escudo: Solo enviamos el voltaje 0 y cerramos si realmente se logró abrir el puerto
    if (Started == 1) {
        FT_STATUS status;
        DWORD bytesTransferidos1 = 0;
        uint8_t buffer[3];
        buffer[0] = 0xAA; 
        buffer[1] = 0x00; 
        buffer[2] = 0x00;        
        
        status = SPI_Write(ftHandle, buffer, 3, &bytesTransferidos1,
                               SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
        status = SPI_CloseChannel(ftHandle);
        APP_CHECK_STATUS(status);
    }
    Cleanup_libMPSSE();
}

#ifdef MATLAB_MEX_FILE 
#include "simulink.c" 
#else
#include "cg_sfun.h" 
#endif
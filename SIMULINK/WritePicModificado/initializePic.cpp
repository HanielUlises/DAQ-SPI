#define S_FUNCTION_NAME  initializePic
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "mex.h"
#include "ftdi_compartido.h"
#include "fixedpoint.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "fixedpoint.h"
#ifdef __cplusplus
}
#endif

static HandleFTDI miHardware;

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, 0); 
    
    if (!ssSetNumInputPorts(S, 0)) return; 
    if (!ssSetNumOutputPorts(S, 1)) return;
    
    ssSetOutputPortWidth(S, 0, 1); 
    
     if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
         DTypeId DataTypeId_0 = ssRegisterDataTypeInteger(S, 0, 64, 0);
         ssSetOutputPortDataType(S, 0, DataTypeId_0);
     } 
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);
    // ======================================================================
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumContStates(S, 0); 
    
    // 2. Number of discrete states: 0
    ssSetNumDiscStates(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
    miHardware.abierto = false; 
    
    // --- NUEVO: Creación del semáforo nativo de Windows ---
    miHardware.semaforo = CreateMutex(NULL, FALSE, NULL);
    // ------------------------------------------------------
    
    Init_libMPSSE();
    
    DWORD channels = 0;
    FT_STATUS status = SPI_GetNumChannels(&channels);
    
    if (status == FT_OK && channels > 0) {
        if (SPI_OpenChannel(0, &miHardware.handle) == FT_OK) {
            ChannelConfig channelConf;
            channelConf.ClockRate = 1000000; 
            channelConf.LatencyTimer = 1;
            channelConf.configOptions = SPI_CONFIG_OPTION_MODE0 | 
                                        SPI_CONFIG_OPTION_CS_DBUS3 | 
                                        SPI_CONFIG_OPTION_CS_ACTIVELOW;
            
            if (SPI_InitChannel(miHardware.handle, &channelConf) == FT_OK) {
                miHardware.abierto = true; 
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

static void mdlOutputs(SimStruct *S, int_T tid) {
    
    uint64_T *llave_salida = (uint64_T *)ssGetOutputPortSignal(S, 0);
    
    llave_salida[0] = (uint64_T)(uintptr_t)(&miHardware);
}

static void mdlTerminate(SimStruct *S) {
    if (miHardware.abierto) { 
        FT_STATUS status;
        DWORD bytesTransferidos1 = 0;
        uint8_t buffer[3];
        buffer[0] = 0xAA; // Cabecera
        buffer[1] = 0x00; // Parte Alta
        buffer[2] = 0x00; // Parte Baja
        
        status = SPI_Write(miHardware.handle, buffer, 3, &bytesTransferidos1,
                            SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                            SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                            SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
        SPI_CloseChannel(miHardware.handle);
        miHardware.abierto = false; 
    }
    Cleanup_libMPSSE();
    
    // --- NUEVO: Destrucción segura del semáforo ---
    if (miHardware.semaforo) {
        CloseHandle(miHardware.semaforo);
        miHardware.semaforo = NULL; // Limpieza por buena práctica
    }
    // ----------------------------------------------
}

#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#include "fixedpoint.c"
#else
#include "cg_sfun.h"
#endif
#define S_FUNCTION_NAME  leerPic
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "ftdi_compartido.h" 
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "fixedpoint.h"
#ifdef __cplusplus
}
#endif


static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, 0); 
    
    // 1. PUERTO DE ENTRADA: La Llave Maestra (64 bits)
    if (!ssSetNumInputPorts(S, 1)) return; 
    
    ssSetInputPortWidth(S, 0, 1); 
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1); 
    
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
        DTypeId DataTypeId_0 = ssRegisterDataTypeInteger(S, 0, 64, 0);
        ssSetInputPortDataType(S, 0, DataTypeId_0);
    }
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);

    // 2. PUERTO DE SALIDA: El valor del encoder (Double)
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE); 
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);
    
    // 3. CONFIGURACIÓN DE MEMORIA NATIVA (DWork)
    if (!ssSetNumDWork(S, 2)) return;

    // DWork 0 -> primer_ciclo (boolean)
    ssSetDWorkWidth(S, 0, 1);
    ssSetDWorkDataType(S, 0, SS_BOOLEAN); 
    ssSetDWorkUsageType(S, 0, SS_DWORK_USED_AS_DWORK);

    // DWork 1 -> offset_encoder (int32)
    ssSetDWorkWidth(S, 1, 1);
    ssSetDWorkDataType(S, 1, SS_INT32);   
    ssSetDWorkUsageType(S, 1, SS_DWORK_USED_AS_DWORK);

    ssSetNumContStates(S, 0); 
    ssSetNumDiscStates(S, 0); 
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S,0,INHERITED_SAMPLE_TIME);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
    real_T *y0 = (real_T *)ssGetOutputPortSignal(S, 0);
    if (y0 != nullptr) {
        y0[0] = 0.0; 
    }
    
    // Extraemos la memoria y le damos sus valores iniciales
    boolean_T *primer_ciclo = (boolean_T *)ssGetDWork(S, 0);
    int32_T *offset_encoder = (int32_T *)ssGetDWork(S, 1);

    primer_ciclo[0] = true;
    offset_encoder[0] = 0;
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    
    if (ssIsMinorTimeStep(S)) {
        return; 
    }

    const uint64_T *llave_entrada = (const uint64_T *)ssGetInputPortSignal(S, 0);
    real_T *y0 = (real_T *)ssGetOutputPortSignal(S, 0);
    
    boolean_T *primer_ciclo = (boolean_T *)ssGetDWork(S, 0);
    int32_T *offset_encoder = (int32_T *)ssGetDWork(S, 1);

    if (llave_entrada == nullptr || y0 == nullptr) {
        return; 
    }

    HandleFTDI *hw = (HandleFTDI *)(uintptr_t)(llave_entrada[0]);

    if (hw == nullptr || !hw->abierto) {
        return; 
    }

    typedef union {
        int32_t valor_32;
        uint8_t bytes[4];
    } Datos8_a_32;

    uint8_t Buf[12];
    DWORD bytesTransferidos = 0;

    // --- NUEVO: Petición del candado a Windows ---
    WaitForSingleObject(hw->semaforo, INFINITE);
    // ---------------------------------------------

    // Ejecutamos la lectura
    FT_STATUS status = SPI_Read(
        hw->handle, 
        Buf, 
        12, 
        &bytesTransferidos,
        SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
        SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
        SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE
    );
    
    // --- NUEVO: Liberación del candado (Inmediatamente después de leer) ---
    ReleaseMutex(hw->semaforo);
    // ----------------------------------------------------------------------

    // Procesamiento de la trama (Esto ya se hace sin bloquear el USB)
    if (status == FT_OK && bytesTransferidos > 5) {
        for (int i = 0; i < (bytesTransferidos - 5); i++) {
            
            // Busqueda de cabecera
            if (Buf[i] == 0xAA && Buf[i+1] == 0x55) {
                
                Datos8_a_32 rxData;
                rxData.bytes[0] = Buf[i+2];
                rxData.bytes[1] = Buf[i+3];
                rxData.bytes[2] = Buf[i+4];
                rxData.bytes[3] = Buf[i+5];
                
                // Reseteo a 0 para primera lectura (usando DWork)
                if (primer_ciclo[0] == true) {
                    offset_encoder[0] = rxData.valor_32; 
                    primer_ciclo[0] = false;                 
                }
                
                int32_t valor_corregido = rxData.valor_32 - offset_encoder[0];
                y0[0] = (real_T)valor_corregido;                    
                
                break; // Rompemos el ciclo al encontrar la primera trama válida
            }
        }
    }
}

static void mdlTerminate(SimStruct *S) {
    // Vacío. El bloque initializePic cierra el puerto FTDI automáticamente.
}

#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#ifdef __cplusplus
extern "C" {
#endif
#include "fixedpoint.c"
#ifdef __cplusplus
}
#endif
#else
#include "cg_sfun.h"
#endif
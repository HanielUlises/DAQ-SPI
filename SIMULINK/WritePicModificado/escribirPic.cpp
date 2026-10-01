#define S_FUNCTION_NAME  escribirPic
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

// DWork 0 -> Guarda el último valor DAC enviado
// DWork 1 -> Indica si ya se realizó el primer envío

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);

    if (!ssSetNumInputPorts(S, 2))
        return;

    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);


    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortRequiredContiguous(S, 1, 1);

    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY)
    {
        DTypeId DataTypeId_1 =
            ssRegisterDataTypeInteger(S, 0, 64, 0);

        ssSetInputPortDataType(S, 1, DataTypeId_1);
    }

    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO);

    if (!ssSetNumOutputPorts(S, 0))
        return;


    // DWORK

    if (!ssSetNumDWork(S, 2))
        return;

    // DWork 0
    // Último valor DAC enviado

    ssSetDWorkWidth(S, 0, 1);
    ssSetDWorkDataType(S, 0, SS_UINT16);
    ssSetDWorkUsageType(S, 0, SS_DWORK_USED_AS_DWORK);

    // DWork 1
    // Indica si ya se realizó el primer envío

    ssSetDWorkWidth(S, 1, 1);
    ssSetDWorkDataType(S, 1, SS_BOOLEAN);
    ssSetDWorkUsageType(S, 1, SS_DWORK_USED_AS_DWORK);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(
        S,
        0,
        INHERITED_SAMPLE_TIME
    );

    ssSetModelReferenceSampleTimeDefaultInheritance(S);

    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START

static void mdlStart(SimStruct *S)
{
    // Obtener DWork 0
    uint16_T *ultimoValor =
        (uint16_T *)ssGetDWork(S, 0);

    // Obtener DWork 1

    boolean_T *primerEnvio =
        (boolean_T *)ssGetDWork(S, 1);

    ultimoValor[0] = 0;

    primerEnvio[0] = true;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{

    if (ssIsMinorTimeStep(S))
    {
        return;
    }

    const real_T *voltaje_ptr =
        (const real_T *)ssGetInputPortSignal(S, 0);

    const uint64_T *llave_entrada =
        (const uint64_T *)ssGetInputPortSignal(S, 1);


    // OBTENER DWORK

    uint16_T *ultimoValor =
        (uint16_T *)ssGetDWork(S, 0);

    boolean_T *primerEnvio =
        (boolean_T *)ssGetDWork(S, 1);


    if (voltaje_ptr == nullptr ||
        llave_entrada == nullptr)
    {
        return;
    }

    HandleFTDI *hw =
        (HandleFTDI *)(uintptr_t)(llave_entrada[0]);


    if (hw == nullptr)
    {
        return;
    }

    if (!hw->abierto)
    {
        return;
    }

    double voltaje = voltaje_ptr[0];

    if (voltaje > 2.5)
    {
        voltaje = 2.5;
    }

    if (voltaje < -2.5)
    {
        voltaje = -2.5;
    }

    uint16_T valor_dac =
        (uint16_T)(13107.0 * (voltaje + 2.5));


    if (!primerEnvio[0] &&
        valor_dac == ultimoValor[0])
    {
        return;
    }

    // --- NUEVO: Petición del candado a Windows ---
    WaitForSingleObject(hw->semaforo, INFINITE);
    // ---------------------------------------------

    if (primerEnvio[0] == true)
    {
        uint8_t bufferLimpieza[3] = {0x00, 0x00, 0x00};
        DWORD bytesLimpieza = 0;
        
        SPI_Write(
            hw->handle,
            bufferLimpieza,
            3,
            &bytesLimpieza,
            SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
            SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
            SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE
        );
    }

    ultimoValor[0] = valor_dac;

    primerEnvio[0] = false;

    uint8_t bufferTx[3];

    bufferTx[0] = 0xAA;

    bufferTx[1] =
        (uint8_t)((valor_dac >> 8) & 0xFF);

    bufferTx[2] =
        (uint8_t)(valor_dac & 0xFF);

    DWORD bytesTransferidos = 0;

    SPI_Write(
        hw->handle,
        bufferTx,
        3,
        &bytesTransferidos,
        SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
        SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
        SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE
    );

    // --- NUEVO: Liberación del candado ---
    ReleaseMutex(hw->semaforo);
    // -------------------------------------
}

static void mdlTerminate(SimStruct *S)
{}

#ifdef MATLAB_MEX_FILE
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
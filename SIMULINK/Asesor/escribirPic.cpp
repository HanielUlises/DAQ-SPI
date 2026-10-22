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

    // --- DAQ-SPI: escala del DAC alimentado con 3.3 V (daq_protocolo.h):
    // 0 V es el código 24824 y no 32767 ---
    uint16_T valor_dac = daq_voltaje_a_dac(voltaje);

    // --- DAQ-SPI: se envía en cada paso aunque el valor no cambie, porque si
    // pasan más de 50 ms sin tramas el dsPIC lleva la salida a 0 V. Una sola
    // trama lleva el voltaje con la salida habilitada; ya no hace falta la
    // cabecera 0xAA ni la trama de limpieza ---
    hw->valorDac = valor_dac;
    hw->salida = true;

    transferenciaPic(hw);
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

/*
 * daqPicEscribir: voltaje de salida para daqPicInicio (ver daq_bloques.h).
 * Sustituye a escribirPic. No usa el FT2232H: deja el valor del paso en el
 * estado compartido y daqPicInicio lo transfiere en su mdlUpdate. El voltaje
 * se envía en cada paso, aunque no cambie, de modo que una trama dañada se
 * corrige en el paso siguiente.
 *
 * Entradas: 1) llave de daqPicInicio;
 *           2) voltaje de salida [V], saturado a [-2.5, 2.5].
 *
 * Debe haber un solo bloque daqPicEscribir por modelo.
 *
 * escribirPic.cpp compila este archivo con DAQ_INTERFAZ_ANTERIOR, con las
 * entradas en el orden de escribirPic: 1) voltaje; 2) llave de initializePic.
 *
 * Compilación: compilar.m
 */
#ifndef S_FUNCTION_NAME
#define S_FUNCTION_NAME  daqPicEscribir
#endif
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "daq_bloques.h"

#ifdef DAQ_INTERFAZ_ANTERIOR
#define ENTRADA_VOLTAJE 0
#define ENTRADA_LLAVE   1
#define MENSAJE_LLAVE   DAQ_NOMBRE ": la entrada 2 debe conectarse a la salida de initializePic."
#else
#define ENTRADA_LLAVE   0
#define ENTRADA_VOLTAJE 1
#define MENSAJE_LLAVE   DAQ_NOMBRE ": la entrada 1 debe conectarse a la salida de daqPicInicio."
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
#endif

    if (!ssSetNumInputPorts(S, 2)) return;
    for (int i = 0; i < 2; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDataType(S, i, SS_DOUBLE);
        ssSetInputPortComplexSignal(S, i, COMPLEX_NO);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    if (!ssSetNumOutputPorts(S, 0)) return;

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    (void)tid;
    if (ssIsMinorTimeStep(S)) {
        return;
    }

    EstadoDaq *e = daq_estado(*ssGetInputPortRealSignal(S, ENTRADA_LLAVE));
    if (e == NULL) {
        ssSetErrorStatus(S, MENSAJE_LLAVE);
        return;
    }

    e->dac = daq_voltaje_a_dac(*ssGetInputPortRealSignal(S, ENTRADA_VOLTAJE));
    e->salida = true;
}

static void mdlTerminate(SimStruct *S)
{
    (void)S;
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

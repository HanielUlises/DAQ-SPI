/*
 * daqPicLeer: posición del encoder y estado del enlace de daqPicInicio (ver
 * daq_bloques.h). Sustituye a leerPic. No usa el FT2232H: entrega lo que
 * recibió daqPicInicio en la transferencia del paso anterior, por lo que la
 * posición tiene un retardo de dos periodos respecto al voltaje aplicado y el
 * bloque no tiene transmisión directa desde el voltaje.
 *
 * Entrada:  llave de daqPicInicio.
 * Salidas:  1) posición del encoder [cuentas], relativa al inicio;
 *           2) errores de comunicación acumulados;
 *           3) pasos atrasados respecto al tiempo real.
 *
 * leerPic.cpp compila este archivo con DAQ_INTERFAZ_ANTERIOR, con la única
 * salida de leerPic, la posición; los errores y los atrasos los reporta
 * initializePic al terminar la simulación.
 *
 * Compilación: compilar.m
 */
#ifndef S_FUNCTION_NAME
#define S_FUNCTION_NAME  daqPicLeer
#endif
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "daq_bloques.h"

#ifdef DAQ_INTERFAZ_ANTERIOR
#define NUM_SALIDAS 1
#else
#define NUM_SALIDAS 3
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
#endif

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    if (!ssSetNumOutputPorts(S, NUM_SALIDAS)) return;
    for (int i = 0; i < NUM_SALIDAS; i++) {
        ssSetOutputPortWidth(S, i, 1);
        ssSetOutputPortDataType(S, i, SS_DOUBLE);
        ssSetOutputPortComplexSignal(S, i, COMPLEX_NO);
    }

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

    const EstadoDaq *e = daq_estado(*ssGetInputPortRealSignal(S, 0));
    if (e == NULL) {
        ssSetErrorStatus(S, DAQ_NOMBRE ": la entrada debe conectarse a la salida de "
                            DAQ_NOMBRE_INICIO ".");
        return;
    }

    ssGetOutputPortRealSignal(S, 0)[0] = e->posicion;
#ifndef DAQ_INTERFAZ_ANTERIOR
    ssGetOutputPortRealSignal(S, 1)[0] = e->errores;
    ssGetOutputPortRealSignal(S, 2)[0] = e->atrasos;
#endif
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

#define S_FUNCTION_NAME  initializePic
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "mex.h"
#include "ftdi_compartido.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "fixedpoint.h"
#ifdef __cplusplus
}
#endif

static HandleFTDI miHardware;
static bool libreriaIniciada = false;

// --- DAQ-SPI: sincronización con el reloj de la PC ---
static LARGE_INTEGER frecuencia;
static LARGE_INTEGER origen;
static bool primerPaso = true;

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
    miHardware.valorDac = DAQ_DAC_CERO;
    miHardware.salida = false;
    miHardware.secuencia = 0;
    miHardware.posicion = 0;

    // --- NUEVO: Creación del semáforo nativo de Windows ---
    miHardware.semaforo = CreateMutex(NULL, FALSE, NULL);
    // ------------------------------------------------------

    Init_libMPSSE();
    libreriaIniciada = true;

    DWORD channels = 0;
    FT_STATUS status = SPI_GetNumChannels(&channels);

    if (status != FT_OK || channels == 0) {
        mexErrMsgTxt("ERROR FTDI: No se detecto ningun dispositivo USB FTDI conectado.");
    }

    // --- DAQ-SPI: abre la interfaz del FT2232H por la que responde el dsPIC
    // con la configuración de siempre (1 MHz, modo 0, CS en DBUS3 activo en
    // bajo). Prueba primero la interfaz A: en algunas PC el canal 0 es la B ---
    DaqCanal canal;
    if (!daq_abrir_canal(channels, DAQ_CANAL_RELOJ_HZ, &miHardware.handle, &canal)) {
        static char mensaje[400];
        snprintf(mensaje, sizeof mensaje,
                 "ERROR FTDI: El dsPIC no respondio (%s). Verifica que tenga cargado el "
                 "firmware dsPicDaq, las conexiones SPI y el cable de GND entre las tarjetas.",
                 canal.detalle);
        mexErrMsgTxt(mensaje);
    }
    miHardware.abierto = true;

    // Primera trama: sincroniza el número de secuencia. Su respuesta trae el
    // eco de las tramas de prueba de daq_abrir_canal y no se cuenta.
    transferenciaPic(&miHardware);
    miHardware.tramas = 0;
    miHardware.errores = 0;

    QueryPerformanceFrequency(&frecuencia);
    primerPaso = true;
}

static void mdlOutputs(SimStruct *S, int_T tid) {

    uint64_T *llave_salida = (uint64_T *)ssGetOutputPortSignal(S, 0);

    llave_salida[0] = (uint64_T)(uintptr_t)(&miHardware);

    // --- DAQ-SPI: cada paso espera a que el reloj de la PC llegue a su tiempo
    // de simulación, para que el periodo de muestreo y el eje del Scope sean
    // tiempo real (si no, Simulink corre lo más rápido que puede). El origen
    // se toma en el primer paso. ---
    LARGE_INTEGER ahora;
    QueryPerformanceCounter(&ahora);
    if (primerPaso) {
        origen = ahora;
        primerPaso = false;
    }
    const double t = ssGetT(S) - ssGetTStart(S);
    const LONGLONG objetivo = origen.QuadPart + (LONGLONG)(t * (double)frecuencia.QuadPart);
    while (ahora.QuadPart < objetivo) {
        QueryPerformanceCounter(&ahora);
    }
}

static void mdlTerminate(SimStruct *S) {
    if (miHardware.abierto) {
        // --- DAQ-SPI: la última trama deja la salida en 0 V (antes se
        // enviaba el código 0x0000, que corresponde a -2.5 V) ---
        miHardware.salida = false;
        transferenciaPic(&miHardware);
        ssPrintf("initializePic: %lu tramas, %lu errores de comunicacion\n",
                 miHardware.tramas, miHardware.errores);
        SPI_CloseChannel(miHardware.handle);
        miHardware.abierto = false;
    }

    // --- DAQ-SPI: Simulink también llama a mdlTerminate sin haber llamado a
    // mdlStart (al evaluar los parámetros del modelo) ---
    if (libreriaIniciada) {
        Cleanup_libMPSSE();
        libreriaIniciada = false;
    }

    // --- NUEVO: Destrucción segura del semáforo ---
    if (miHardware.semaforo) {
        CloseHandle(miHardware.semaforo);
        miHardware.semaforo = NULL; // Limpieza por buena práctica
    }
    // ----------------------------------------------
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

/*
 * daqPic: bloque único de entrada/salida para la DAQ basada en dsPIC
 * (etapa 3 de docs/propuesta.pdf). Requiere el firmware dsPicDaq.
 *
 * Entrada:  voltaje de salida [V], saturado a [-2.5, 2.5].
 * Salidas:  1) posición del encoder [cuentas], relativa al inicio;
 *           2) errores de comunicación acumulados;
 *           3) pasos atrasados respecto al tiempo real.
 * Parámetros: periodo de muestreo Ts [s]; sincronizar con tiempo real (0/1).
 *
 * Cada paso ejecuta una sola transferencia full-duplex en mdlUpdate, con el
 * voltaje del paso actual. La posición recibida se entrega en el paso
 * siguiente, por lo que el bloque no tiene transmisión directa y puede usarse
 * en lazo cerrado sin generar lazos algebraicos. La posición corresponde a la
 * transferencia anterior: el retardo total del lazo es de dos periodos.
 *
 * El bloque abre y cierra el FT2232H; no requiere initializePic. Al terminar
 * la simulación envía una trama con la salida deshabilitada (0 V).
 *
 * Compilación: compilar.m
 */
#define S_FUNCTION_NAME  daqPic
#define S_FUNCTION_LEVEL 2

#define NOMINMAX
#include "simstruc.h"
#include <windows.h>
#include <stdint.h>
#include <string.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"
#include "../../protocolo/daq_protocolo.h"

#define PARAM_TS            0
#define PARAM_TIEMPO_REAL   1
#define NUM_PARAMS          2

static const DWORD kOpciones = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

static const uint8_t kStatusErrores = DAQ_STATUS_ERR_CRC | DAQ_STATUS_ERR_INICIO |
                                      DAQ_STATUS_ERR_LONGITUD | DAQ_STATUS_VIGILANCIA;

struct Estado {
    bool libreria;          /* Init_libMPSSE ejecutado */
    bool abierto;           /* canal SPI abierto */
    FT_HANDLE handle;
    uint8_t seq;            /* número de secuencia de la siguiente trama */
    uint8_t seq_anterior;   /* número de secuencia de la última trama enviada */
    double posicion;
    double errores;
    double atrasos;
    double ts;
    bool tiempo_real;
    LARGE_INTEGER frecuencia;
    LARGE_INTEGER origen;
    uint32_t paso;
};

/* Transfiere una trama y valida la respuesta. Devuelve true si la respuesta
 * es válida; en ese caso actualiza la posición. */
static bool transferir(Estado *e, uint16_t dac, uint8_t flags, uint8_t *status)
{
    uint8_t tx[DAQ_TRAMA_LEN];
    uint8_t rx[DAQ_TRAMA_LEN];
    DWORD n = 0;
    uint8_t seq_eco;
    int32_t pos;

    daq_armar_trama_pc(tx, e->seq, dac, flags);
    FT_STATUS st = SPI_ReadWrite(e->handle, rx, tx, DAQ_TRAMA_LEN, &n, kOpciones);

    const uint8_t enviado_antes = e->seq_anterior;
    e->seq_anterior = e->seq;
    e->seq++;

    if (st != FT_OK || n != DAQ_TRAMA_LEN) {
        return false;
    }
    if (daq_leer_trama_pic(rx, &seq_eco, &pos, status) != 0) {
        return false;
    }
    e->posicion = (double)pos;
    return seq_eco == enviado_antes;
}

/* ------------------------------------------------------------------------- */

#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S)
{
    const mxArray *ts = ssGetSFcnParam(S, PARAM_TS);
    const mxArray *tr = ssGetSFcnParam(S, PARAM_TIEMPO_REAL);

    if (!mxIsDouble(ts) || mxGetNumberOfElements(ts) != 1 || mxGetScalar(ts) <= 0.0) {
        ssSetErrorStatus(S, "daqPic: Ts debe ser un escalar positivo.");
        return;
    }
    if (!mxIsDouble(tr) || mxGetNumberOfElements(tr) != 1) {
        ssSetErrorStatus(S, "daqPic: el segundo parametro debe ser 0 o 1.");
        return;
    }
}

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) {
        return;
    }
#endif
    ssSetSFcnParamTunable(S, PARAM_TS, SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, PARAM_TIEMPO_REAL, SS_PRM_NOT_TUNABLE);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 3)) return;
    for (int i = 0; i < 3; i++) {
        ssSetOutputPortWidth(S, i, 1);
        ssSetOutputPortDataType(S, i, SS_DOUBLE);
        ssSetOutputPortComplexSignal(S, i, COMPLEX_NO);
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_CALL_TERMINATE_ON_EXIT);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, PARAM_TS)));
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    Estado *e = new Estado();
    ssGetPWork(S)[0] = e;

    e->ts = mxGetScalar(ssGetSFcnParam(S, PARAM_TS));
    e->tiempo_real = mxGetScalar(ssGetSFcnParam(S, PARAM_TIEMPO_REAL)) != 0.0;

    Init_libMPSSE();
    e->libreria = true;

    DWORD canales = 0;
    if (SPI_GetNumChannels(&canales) != FT_OK || canales == 0) {
        ssSetErrorStatus(S, "daqPic: no se detecto ningun FT2232H.");
        return;
    }
    if (SPI_OpenChannel(0, &e->handle) != FT_OK) {
        ssSetErrorStatus(S, "daqPic: no se pudo abrir el canal 0 del FT2232H "
                            "(lo usa otro programa o modelo?).");
        return;
    }
    e->abierto = true;

    ChannelConfig conf;
    memset(&conf, 0, sizeof conf);
    conf.ClockRate = 1000000;
    conf.LatencyTimer = 1;
    conf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 |
                         SPI_CONFIG_OPTION_CS_ACTIVELOW;
    if (SPI_InitChannel(e->handle, &conf) != FT_OK) {
        ssSetErrorStatus(S, "daqPic: fallo la inicializacion del canal SPI.");
        return;
    }

    /* Primera trama: pone en cero el encoder con la salida deshabilitada. Su
     * respuesta refleja el estado previo y se descarta. La segunda confirma
     * que el dsPIC responde con el protocolo nuevo y entrega la posición 0. */
    uint8_t status = 0;
    (void)transferir(e, DAQ_DAC_CERO, DAQ_FLAG_RESET_ENC, &status);
    if (!transferir(e, DAQ_DAC_CERO, 0u, &status)) {
        ssSetErrorStatus(S, "daqPic: el dsPIC no respondio con el protocolo esperado. "
                            "Verifique que tenga cargado el firmware dsPicDaq y las "
                            "conexiones SPI.");
        return;
    }

    QueryPerformanceFrequency(&e->frecuencia);
    QueryPerformanceCounter(&e->origen);
    e->paso = 0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    (void)tid;
    const Estado *e = (const Estado *)ssGetPWork(S)[0];
    real_T *posicion = ssGetOutputPortRealSignal(S, 0);
    real_T *errores = ssGetOutputPortRealSignal(S, 1);
    real_T *atrasos = ssGetOutputPortRealSignal(S, 2);

    posicion[0] = e ? e->posicion : 0.0;
    errores[0] = e ? e->errores : 0.0;
    atrasos[0] = e ? e->atrasos : 0.0;
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
    (void)tid;
    Estado *e = (Estado *)ssGetPWork(S)[0];
    if (e == NULL || !e->abierto) {
        return;
    }

    /* Sincronización con el reloj de la PC */
    if (e->tiempo_real) {
        const double cuentas_ts = e->ts * (double)e->frecuencia.QuadPart;
        const LONGLONG objetivo = e->origen.QuadPart + (LONGLONG)(e->paso * cuentas_ts);
        LARGE_INTEGER ahora;
        QueryPerformanceCounter(&ahora);
        if ((double)(ahora.QuadPart - objetivo) > cuentas_ts) {
            /* Más de un periodo de atraso: se cuenta y se reajusta el origen
             * para no intentar recuperar los pasos perdidos en ráfaga. */
            e->atrasos += 1.0;
            e->origen.QuadPart = ahora.QuadPart - (LONGLONG)(e->paso * cuentas_ts);
        } else {
            while (ahora.QuadPart < objetivo) {
                QueryPerformanceCounter(&ahora);
            }
        }
    }
    e->paso++;

    const uint16_t dac = daq_voltaje_a_dac(*ssGetInputPortRealSignal(S, 0));

    uint8_t status = 0;
    if (!transferir(e, dac, DAQ_FLAG_SALIDA_HAB, &status) || (status & kStatusErrores)) {
        e->errores += 1.0;
    }
}

static void mdlTerminate(SimStruct *S)
{
    Estado *e = (Estado *)ssGetPWork(S)[0];
    if (e == NULL) {
        return;
    }
    if (e->abierto) {
        uint8_t status = 0;
        (void)transferir(e, DAQ_DAC_CERO, 0u, &status);
        SPI_CloseChannel(e->handle);
        e->abierto = false;
    }
    if (e->libreria) {
        Cleanup_libMPSSE();
    }
    delete e;
    ssGetPWork(S)[0] = NULL;
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

/*
 * daqPicInicio: abre el FT2232H y ejecuta la transferencia de cada paso para
 * los bloques daqPicEscribir y daqPicLeer (ver daq_bloques.h). Requiere el
 * firmware dsPicDaq. Sustituye a initializePic; no debe combinarse con daqPic
 * ni con los bloques anteriores en el mismo modelo.
 *
 * Salida:     llave para daqPicEscribir y daqPicLeer.
 * Parámetros: periodo de muestreo Ts [s]; sincronizar con tiempo real (0/1).
 *
 * Al iniciar pone en cero el encoder y verifica que el dsPIC responda con el
 * protocolo nuevo. Al terminar la simulación envía una trama con la salida
 * deshabilitada (0 V). Sin un bloque daqPicEscribir en el modelo, la salida
 * permanece deshabilitada.
 *
 * Compilación: compilar.m
 */
#define S_FUNCTION_NAME  daqPicInicio
#define S_FUNCTION_LEVEL 2

#define NOMINMAX
#include "simstruc.h"
#include <windows.h>
#include <stdint.h>
#include <string.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"
#include "daq_bloques.h"

#define PARAM_TS            0
#define PARAM_TIEMPO_REAL   1
#define NUM_PARAMS          2

static const DWORD kOpciones = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

static const uint8_t kStatusErrores = DAQ_STATUS_ERR_CRC | DAQ_STATUS_ERR_INICIO |
                                      DAQ_STATUS_ERR_LONGITUD | DAQ_STATUS_VIGILANCIA |
                                      DAQ_STATUS_REINICIO;

struct Estado {
    EstadoDaq comun;        /* lo que leen y escriben los otros bloques */
    bool libreria;          /* Init_libMPSSE ejecutado */
    bool abierto;           /* canal SPI abierto */
    FT_HANDLE handle;
    uint8_t seq;            /* número de secuencia de la siguiente trama */
    uint8_t seq_anterior;   /* número de secuencia de la última trama enviada */
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
    e->comun.posicion = (double)pos;
    return seq_eco == enviado_antes;
}

/* ------------------------------------------------------------------------- */

#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S)
{
    const mxArray *ts = ssGetSFcnParam(S, PARAM_TS);
    const mxArray *tr = ssGetSFcnParam(S, PARAM_TIEMPO_REAL);

    if (!mxIsDouble(ts) || mxGetNumberOfElements(ts) != 1 || mxGetScalar(ts) <= 0.0) {
        ssSetErrorStatus(S, "daqPicInicio: Ts debe ser un escalar positivo.");
        return;
    }
    if (!mxIsDouble(tr) || mxGetNumberOfElements(tr) != 1) {
        ssSetErrorStatus(S, "daqPicInicio: el segundo parametro debe ser 0 o 1.");
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

    if (!ssSetNumInputPorts(S, 0)) return;

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);

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

    e->comun.dac = DAQ_DAC_CERO;
    e->ts = mxGetScalar(ssGetSFcnParam(S, PARAM_TS));
    e->tiempo_real = mxGetScalar(ssGetSFcnParam(S, PARAM_TIEMPO_REAL)) != 0.0;

    if ((uintptr_t)daq_llave(&e->comun) != (uintptr_t)&e->comun) {
        ssSetErrorStatus(S, "daqPicInicio: la direccion del estado no cabe en un double.");
        return;
    }

    Init_libMPSSE();
    e->libreria = true;

    DWORD canales = 0;
    if (SPI_GetNumChannels(&canales) != FT_OK || canales == 0) {
        ssSetErrorStatus(S, "daqPicInicio: no se detecto ningun FT2232H.");
        return;
    }
    if (SPI_OpenChannel(0, &e->handle) != FT_OK) {
        ssSetErrorStatus(S, "daqPicInicio: no se pudo abrir el canal 0 del FT2232H "
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
        ssSetErrorStatus(S, "daqPicInicio: fallo la inicializacion del canal SPI.");
        return;
    }

    /* Primera trama: pone en cero el encoder con la salida deshabilitada. Su
     * respuesta refleja el estado previo y se descarta. La segunda confirma
     * que el dsPIC responde con el protocolo nuevo y entrega la posición 0. */
    uint8_t status = 0;
    (void)transferir(e, DAQ_DAC_CERO, DAQ_FLAG_RESET_ENC, &status);
    if (!transferir(e, DAQ_DAC_CERO, 0u, &status)) {
        ssSetErrorStatus(S, "daqPicInicio: el dsPIC no respondio con el protocolo esperado. "
                            "Verifique que tenga cargado el firmware dsPicDaq y las "
                            "conexiones SPI.");
        return;
    }

    QueryPerformanceFrequency(&e->frecuencia);
    e->paso = 0;

    /* Sólo a partir de aquí los otros bloques aceptan la llave */
    e->comun.magia = DAQ_BLOQUES_MAGIA;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    (void)tid;
    Estado *e = (Estado *)ssGetPWork(S)[0];
    real_T *llave = ssGetOutputPortRealSignal(S, 0);

    llave[0] = e ? daq_llave(&e->comun) : 0.0;
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
    (void)tid;
    Estado *e = (Estado *)ssGetPWork(S)[0];
    if (e == NULL || !e->abierto) {
        return;
    }

    /* Sincronización con el reloj de la PC. El origen se toma en el primer
     * paso y no en mdlStart: la inicialización del resto del modelo no cuenta
     * como atraso. */
    if (e->tiempo_real && e->paso == 0) {
        QueryPerformanceCounter(&e->origen);
    } else if (e->tiempo_real) {
        const double cuentas_ts = e->ts * (double)e->frecuencia.QuadPart;
        const LONGLONG objetivo = e->origen.QuadPart + (LONGLONG)(e->paso * cuentas_ts);
        LARGE_INTEGER ahora;
        QueryPerformanceCounter(&ahora);
        if ((double)(ahora.QuadPart - objetivo) > cuentas_ts) {
            /* Más de un periodo de atraso: se cuenta y se reajusta el origen
             * para no intentar recuperar los pasos perdidos en ráfaga. */
            e->comun.atrasos += 1.0;
            e->origen.QuadPart = ahora.QuadPart - (LONGLONG)(e->paso * cuentas_ts);
        } else {
            while (ahora.QuadPart < objetivo) {
                QueryPerformanceCounter(&ahora);
            }
        }
    }
    e->paso++;

    /* Entre mdlStart y el primer paso Simulink inicializa el resto del modelo,
     * y si tarda más de 50 ms expira la vigilancia del dsPIC. La bandera llega
     * en la respuesta del segundo paso y no indica un error del enlace. */
    uint8_t errores = kStatusErrores;
    if (e->paso == 2) {
        errores &= (uint8_t)~DAQ_STATUS_VIGILANCIA;
    }

    const uint16_t dac = e->comun.salida ? e->comun.dac : DAQ_DAC_CERO;
    const uint8_t flags = e->comun.salida ? DAQ_FLAG_SALIDA_HAB : 0u;

    uint8_t status = 0;
    if (!transferir(e, dac, flags, &status) || (status & errores)) {
        e->comun.errores += 1.0;
    }
}

static void mdlTerminate(SimStruct *S)
{
    Estado *e = (Estado *)ssGetPWork(S)[0];
    if (e == NULL) {
        return;
    }
    e->comun.magia = 0u;
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

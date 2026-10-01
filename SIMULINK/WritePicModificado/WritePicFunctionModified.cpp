
#define S_FUNCTION_LEVEL               2
#define S_FUNCTION_NAME                WritePicFunction

/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
/* %%%-SFUNWIZ_defines_Changes_BEGIN --- EDIT HERE TO _END */
#define NUM_INPUTS                     1

/* Input Port  0 */
#define IN_PORT_0_NAME                 u0
#define INPUT_0_DIMS_ND                {1,1}
#define INPUT_0_NUM_ELEMS              1
#define INPUT_0_WIDTH                  1
#define INPUT_DIMS_0_COL               1
#define INPUT_0_DTYPE                  real_T
#define INPUT_0_COMPLEX                COMPLEX_NO
#define INPUT_0_UNIT                   ""
#define IN_0_BUS_BASED                 0
#define IN_0_BUS_NAME
#define IN_0_DIMS                      1-D
#define INPUT_0_FEEDTHROUGH            1
#define IN_0_ISSIGNED                  1
#define IN_0_WORDLENGTH                8
#define IN_0_FIXPOINTSCALING           1
#define IN_0_FRACTIONLENGTH            3
#define IN_0_BIAS                      0
#define IN_0_SLOPE                     0.125
#define NUM_OUTPUTS                    1

/* Output Port  0 */
#define OUT_PORT_0_NAME                y0
#define OUTPUT_0_DIMS_ND               {1,1}
#define OUTPUT_0_NUM_ELEMS             1
#define OUTPUT_0_WIDTH                 1
#define OUTPUT_DIMS_0_COL              1
#define OUTPUT_0_DTYPE                 uint16_T
#define OUTPUT_0_COMPLEX               COMPLEX_NO
#define OUTPUT_0_UNIT                  ""
#define OUT_0_BUS_BASED                0
#define OUT_0_BUS_NAME
#define OUT_0_DIMS                     1-D
#define OUT_0_ISSIGNED                 1
#define OUT_0_WORDLENGTH               8
#define OUT_0_FIXPOINTSCALING          1
#define OUT_0_FRACTIONLENGTH           3
#define OUT_0_BIAS                     0
#define OUT_0_SLOPE                    0.125
#define NPARAMS                        0
#define SAMPLE_TIME_0                  INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES                0
#define DISC_STATES_IC                 [0]
#define NUM_CONT_STATES                0
#define CONT_STATES_IC                 [0]
#define SFUNWIZ_GENERATE_TLC           1
#define SOURCEFILES                    "__SFB__"
#define PANELINDEX                     N/A
#define USE_SIMSTRUCT                  0
#define SHOW_COMPILE_STEPS             0
#define CREATE_DEBUG_MEXFILE           0
#define SAVE_CODE_ONLY                 0
#define SFUNWIZ_REVISION               3.0

/* %%%-SFUNWIZ_defines_Changes_END --- EDIT HERE TO _BEGIN */
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
#include "simstruc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <windows.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"
#include "mex.h"

// extern "C" void WritePicFunction_Start_wrapper(void);
// extern "C" void WritePicFunction_Outputs_wrapper(const real_T *u0,
//   uint16_T *y0);
// extern "C" void WritePicFunction_Terminate_wrapper(void);
FT_HANDLE ftHandle;
int Started = 0;

int primer_ciclo = 1;
int32_t offset_encoder = 0;
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1
/*====================*
 * S-function methods *
 *====================*/
/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S, NPARAMS);
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    return;                            /* Parameter mismatch will be reported by Simulink */
  }

  ssSetArrayLayoutForCodeGen(S, SS_COLUMN_MAJOR);
  ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);
  ssSetNumContStates(S, NUM_CONT_STATES);
  ssSetNumDiscStates(S, NUM_DISC_STATES);
  if (!ssSetNumInputPorts(S, NUM_INPUTS))
    return;

  /* Input Port 0 */
  ssSetInputPortWidth(S, 0, INPUT_0_NUM_ELEMS);
  ssSetInputPortDataType(S, 0, SS_DOUBLE);
  ssSetInputPortComplexSignal(S, 0, INPUT_0_COMPLEX);
  ssSetInputPortDirectFeedThrough(S, 0, INPUT_0_FEEDTHROUGH);
  ssSetInputPortRequiredContiguous(S, 0, 1);/*direct input signal access*/

  /*
   * Configure the Units for Input Ports
   */
  if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {

#if defined(MATLAB_MEX_FILE)

    UnitId inUnitIdReg;
    ssRegisterUnitFromExpr(S, INPUT_0_UNIT, &inUnitIdReg);
    if (inUnitIdReg != INVALID_UNIT_ID) {
      ssSetInputPortUnit(S, 0, inUnitIdReg);
    } else {
      ssSetLocalErrorStatus(S,
                            "Invalid Unit provided for input port u0 of S-Function WritePicFunction");
      return;
    }

#endif

  }

  if (!ssSetNumOutputPorts(S, NUM_OUTPUTS))
    return;

  /* Output Port 0 */
  ssSetOutputPortWidth(S, 0, OUTPUT_0_NUM_ELEMS);
  ssSetOutputPortDataType(S, 0, SS_UINT16);
  ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);

  /*
   * Configure the Units for Output Ports
   */
  if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {

#if defined(MATLAB_MEX_FILE)

    UnitId outUnitIdReg;
    ssRegisterUnitFromExpr(S, OUTPUT_0_UNIT, &outUnitIdReg);
    if (outUnitIdReg != INVALID_UNIT_ID) {
      ssSetOutputPortUnit(S, 0, outUnitIdReg);
    } else {
      ssSetLocalErrorStatus(S,
                            "Invalid Unit provided for output port y0 of S-Function WritePicFunction");
      return;
    }

#endif

  }

  ssSetNumPWork(S, 0);
  ssSetNumSampleTimes(S, 1);
  ssSetNumRWork(S, 0);
  ssSetNumIWork(S, 0);
  ssSetNumModes(S, 0);
  ssSetNumNonsampledZCs(S, 0);
  ssSetSimulinkVersionGeneratedIn(S, "25.1");

  /* Take care when specifying exception free code - see sfuntmpl_doc.c */
  ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_FALSE);
  ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                   SS_OPTION_USE_TLC_WITH_ACCELERATOR |
                   SS_OPTION_WORKS_WITH_CODE_REUSE));
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S,
  int_T port,
  const DimsInfo_T *dimsInfo)
{
  if (!ssSetInputPortDimensionInfo(S, port, dimsInfo))
    return;
}

#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)

static void mdlSetOutputPortDimensionInfo(SimStruct *S,
  int_T port,
  const DimsInfo_T *dimsInfo)
{
  if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo))
    return;
}

#endif

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, SAMPLE_TIME_0);
  ssSetModelReferenceSampleTimeDefaultInheritance(S);
  ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE

static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType)
{
  ssSetInputPortDataType(S, 0, dType);
}

#define MDL_SET_OUTPUT_PORT_DATA_TYPE

static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)
{
  ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES

static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
  ssSetInputPortDataType(S, 0, SS_DOUBLE);
  ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}

#define MDL_START                                                /* Change to #undef to remove function */
#if defined(MDL_START)

/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
  Started = 0;

primer_ciclo = 1;
offset_encoder = 0;

ChannelConfig channelConf;
DWORD channel;
DWORD channels;
FT_STATUS status;
    
    // Configuraci�n para leer del dsPIC
    channelConf.ClockRate = 1000000;
    channelConf.LatencyTimer = 1;
    channelConf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;

    Init_libMPSSE();
    status = SPI_GetNumChannels(&channels);

    if (channels > 0)
    {
        channel = 0; 
        status = SPI_OpenChannel(channel, &ftHandle);
        if (status == FT_OK)
        {
            status = SPI_InitChannel(ftHandle, &channelConf);
            if (status == FT_OK)
            {
                Started = 1; 
            }
            else 
            {
                printf("ERROR FTDI: Se encontro el chip pero fallo la inicializacion SPI.\n");
            }
        }
        else 
        {
            printf("ERROR FTDI: No se pudo abrir el puerto. Verifica que no este en uso.\n");
        }
    }
    else 
    {
        printf("ERROR FTDI: No se detecto ningun dispositivo USB FTDI conectado.\n");
    }
}

#endif                                 /*  MDL_START */

/* Function: mdlOutputs =======================================================
 *
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
  const real_T *u0 = (real_T *) ssGetInputPortRealSignal(S, 0);
  uint16_T *y0 = (uint16_T *) ssGetOutputPortRealSignal(S, 0);
  FT_STATUS status;
    DWORD bytesTransferidos = 0;

    if (Started == 1)
    {
        
        double voltaje = u0[0];
        if (voltaje > 2.5) {
            voltaje = 2.5; // L�mite m�ximo
        }else if (voltaje < -2.5){
            voltaje = -2.5; //Limite minimo
        }

        //-2.5 = 0, 0 = 32767, 2.5 = 65535
        uint16_t valor_dac = (uint16_t)(13107.0 * (voltaje + 2.5));

        
        uint8_t bufferTx[3];
        bufferTx[0] = 0xAA; // Cabecera
        bufferTx[1] = (valor_dac >> 8) & 0xFF; // Parte Alta
        bufferTx[2] = valor_dac & 0xFF;        // Parte Baja
        
        status = SPI_Write(ftHandle, bufferTx, 3, &bytesTransferidos,
                               SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                               SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
                              
        y0[0] = (uint16_T)valor_dac; 
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
  FT_STATUS status;
    DWORD bytesTransferidos1 = 0;
    uint8_t buffer[3];
    buffer[0] = 0xAA; // Cabecera
    buffer[1] = 0x00; // Parte Alta
    buffer[2] = 0x00;        // Parte Baja
    
    status = SPI_Write(ftHandle, buffer, 3, &bytesTransferidos1,
                           SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
                           SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                           SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
    status = SPI_CloseChannel(ftHandle);
    APP_CHECK_STATUS(status);
    Cleanup_libMPSSE();
}

#ifdef MATLAB_MEX_FILE                 /* Is this file being compiled as a MEX-file? */
#include "simulink.c"                  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"                   /* Code generation registration function */
#endif

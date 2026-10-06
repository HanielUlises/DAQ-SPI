/* Imitación mínima de simstruc.h para ejecutar las S-Functions fuera de MATLAB */
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
typedef double real_T; typedef int int_T; typedef bool boolean_T;
struct mxArray { double v; };
static inline bool mxIsDouble(const mxArray *) { return true; }
static inline size_t mxGetNumberOfElements(const mxArray *) { return 1; }
static inline double mxGetScalar(const mxArray *a) { return a->v; }
enum { SS_DOUBLE = 0, COMPLEX_NO = 0, SS_PRM_NOT_TUNABLE = 0 };
#define SS_OPTION_EXCEPTION_FREE_CODE 1u
#define SS_OPTION_CALL_TERMINATE_ON_EXIT 2u
#define INHERITED_SAMPLE_TIME (-1.0)
struct SimStruct {
    mxArray params[4];
    const char *error;
    int nin, nout;
    double in[4][1];
    double out[4][1];
    void *pwork[2];
    double ts;
};
#define ssSetNumSFcnParams(S, n) ((void)0)
#define ssGetSFcnParam(S, i) (&(S)->params[i])
#define ssSetErrorStatus(S, m) ((S)->error = (m))
#define ssGetErrorStatus(S) ((S)->error)
#define ssSetSFcnParamTunable(S, i, t) ((void)0)
#define ssSetNumInputPorts(S, n) ((S)->nin = (n), 1)
#define ssSetNumOutputPorts(S, n) ((S)->nout = (n), 1)
#define ssSetInputPortWidth(S, i, w) ((void)0)
#define ssSetInputPortDataType(S, i, t) ((void)0)
#define ssSetInputPortComplexSignal(S, i, c) ((void)0)
#define ssSetInputPortDirectFeedThrough(S, i, d) ((void)0)
#define ssSetInputPortRequiredContiguous(S, i, c) ((void)0)
#define ssSetOutputPortWidth(S, i, w) ((void)0)
#define ssSetOutputPortDataType(S, i, t) ((void)0)
#define ssSetOutputPortComplexSignal(S, i, c) ((void)0)
#define ssSetNumContStates(S, n) ((void)0)
#define ssSetNumDiscStates(S, n) ((void)0)
#define ssSetNumSampleTimes(S, n) ((void)0)
#define ssSetNumPWork(S, n) ((void)0)
#define ssSetOptions(S, o) ((void)0)
#define ssSetSampleTime(S, i, t) ((S)->ts = (t))
#define ssSetOffsetTime(S, i, t) ((void)0)
#define ssGetPWork(S) ((S)->pwork)
#define ssGetOutputPortRealSignal(S, i) ((S)->out[i])
#define ssGetInputPortRealSignal(S, i) ((const real_T *)(S)->in[i])
#define ssIsMinorTimeStep(S) false

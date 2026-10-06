/* Exporta las funciones de la S-Function para el arnés */
#include "arnes.h"
#define ARNES_CAT2(a, b) a##b
#define ARNES_CAT(a, b) ARNES_CAT2(a, b)
#define ARNES_STR2(a) #a
#define ARNES_STR(a) ARNES_STR2(a)
SFuncion ARNES_CAT(sfun_, S_FUNCTION_NAME) = {
    ARNES_STR(S_FUNCTION_NAME), mdlInitializeSizes, mdlInitializeSampleTimes,
#ifdef MDL_START
    mdlStart,
#else
    nullptr,
#endif
    mdlOutputs,
#ifdef MDL_UPDATE
    mdlUpdate,
#else
    nullptr,
#endif
    mdlTerminate};

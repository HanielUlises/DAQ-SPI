#pragma once
#include "simstruc.h"
struct SFuncion {
    const char *nombre;
    void (*tamanos)(SimStruct *);
    void (*tiempos)(SimStruct *);
    void (*inicio)(SimStruct *);
    void (*salidas)(SimStruct *, int_T);
    void (*actualizar)(SimStruct *, int_T);
    void (*terminar)(SimStruct *);
};

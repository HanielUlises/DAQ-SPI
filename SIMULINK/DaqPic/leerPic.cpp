/*
 * leerPic: daqPicLeer con la interfaz del bloque anterior.
 *
 * Entrada: llave de initializePic.
 * Salida:  posición del encoder [cuentas], relativa al inicio, con un retardo
 *          de dos periodos respecto al voltaje aplicado.
 *
 * A diferencia del bloque anterior, no usa el FT2232H: entrega la posición que
 * recibió initializePic en la transferencia del paso anterior.
 *
 * Compilación: compilar.m
 */
#define S_FUNCTION_NAME  leerPic
#define DAQ_INTERFAZ_ANTERIOR
#include "daqPicLeer.cpp"

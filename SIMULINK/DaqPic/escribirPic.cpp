/*
 * escribirPic: daqPicEscribir con la interfaz del bloque anterior.
 *
 * Entradas: 1) voltaje de salida [V], saturado a [-2.5, 2.5];
 *           2) llave de initializePic.
 *
 * A diferencia del bloque anterior, no usa el FT2232H: initializePic envía el
 * voltaje y recibe la posición en la misma transferencia de cada paso.
 *
 * Compilación: compilar.m
 */
#define S_FUNCTION_NAME  escribirPic
#define DAQ_INTERFAZ_ANTERIOR
#include "daqPicEscribir.cpp"

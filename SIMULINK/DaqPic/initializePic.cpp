/*
 * initializePic: daqPicInicio con la interfaz del bloque anterior, para usar
 * los modelos existentes (MergedPic.slx, ReadPicModificado.slx,
 * WritePicModificado.slx) sin cambiarlos. Requiere el firmware dsPicDaq.
 *
 * Salida: llave para escribirPic y leerPic.
 *
 * No tiene parámetros: el periodo se hereda del modelo, que debe usar un
 * solver de paso fijo, y la simulación siempre se sincroniza con el reloj de
 * la PC. Al terminar escribe en la ventana de comandos los pasos, los errores
 * de comunicación y los pasos atrasados.
 *
 * Compilación: compilar.m
 */
#define S_FUNCTION_NAME  initializePic
#define DAQ_INTERFAZ_ANTERIOR
#include "daqPicInicio.cpp"

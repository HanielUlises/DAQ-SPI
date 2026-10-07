/*
 * Diagrama de bloques del modelo que ejecuta el visor (control.h), dibujado
 * al estilo de Simulink. No se edita: cambia entre lazo abierto y lazo
 * cerrado, y un clic sobre un bloque lo selecciona para mostrar sus
 * parámetros. Con la prueba en curso las líneas llevan el valor actual de
 * cada señal.
 */
#ifndef DIAGRAMA_H
#define DIAGRAMA_H

#include "control.h"

enum Bloque {
    BLOQUE_FUENTE,          /* fuente en lazo abierto, referencia en lazo cerrado */
    BLOQUE_SUMA,
    BLOQUE_PID,
    BLOQUE_SATURACION,
    BLOQUE_DAQ,
    BLOQUE_SCOPE,
    BLOQUE_ERRORES,
    BLOQUE_ATRASOS,
    BLOQUE_NUM,
    BLOQUE_NINGUNO = BLOQUE_NUM,
};

/* Nombre del bloque en el diagrama y en el inspector */
const char *nombre_bloque(Bloque b, bool cerrado);

struct DatosDiagrama {
    bool cerrado = false;
    Forma forma = FORMA_NADA;
    const char *modo = "";              /* texto bajo daqPic: dsPIC, lazo, simulado */
    unsigned long errores = 0, atrasos = 0;
    bool hay_valores = false;           /* hay al menos un paso con la salida habilitada */
    double r = 0.0, e = 0.0, u = 0.0, y = 0.0;    /* r y e: NAN si no hay referencia */
    const char *unidad = "";
};

/* Altura que ocupa el diagrama con un ancho dado */
float alto_diagrama(float ancho);

/* Dibuja el diagrama en la posición actual con el ancho disponible y
 * devuelve el bloque sobre el que se hizo clic (BLOQUE_NINGUNO si ninguno) */
Bloque diagrama(const DatosDiagrama &d, Bloque seleccion);

#endif /* DIAGRAMA_H */

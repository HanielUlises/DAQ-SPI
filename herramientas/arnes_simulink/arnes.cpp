/* Ejecuta las S-Functions de SIMULINK/DaqPic como lo haría Simulink, contra la
 * tarjeta real: mdlInitializeSizes, mdlInitializeSampleTimes, mdlStart, una
 * pausa de inicialización, luego por paso todos los mdlOutputs y después
 * todos los mdlUpdate, y al final mdlTerminate. */
#include <cmath>
#include <cstdio>
#include <unistd.h>
#include "simulacion/arnes.h"
#include "../../SIMULINK/DaqPic/daq_bloques.h"

extern SFuncion sfun_daqPic, sfun_daqPicInicio, sfun_daqPicEscribir, sfun_daqPicLeer;

static int fallas = 0;
static void revisar(bool ok, const char *que)
{
    printf("  [%s] %s\n", ok ? "ok" : "FALLA", que);
    if (!ok) fallas++;
}

static void preparar(SFuncion &f, SimStruct &S, double ts, double tr)
{
    memset(&S, 0, sizeof S);
    S.params[0].v = ts;
    S.params[1].v = tr;
    f.tamanos(&S);
    f.tiempos(&S);
}

static double voltaje(int k) { return 0.5 * sin(2.0 * M_PI * 0.5 * k * 0.001); }

static void prueba_daqpic(int pasos)
{
    printf("daqPic, %d pasos a 1 kHz, 300 ms entre mdlStart y el primer paso\n", pasos);
    SimStruct S;
    preparar(sfun_daqPic, S, 0.001, 1.0);
    revisar(S.ts == 0.001 && S.nin == 1 && S.nout == 3, "puertos y periodo");
    sfun_daqPic.inicio(&S);
    revisar(S.error == NULL, S.error ? S.error : "mdlStart sin error");
    if (S.error) { sfun_daqPic.terminar(&S); return; }
    usleep(300000);
    double atrasos_paso1 = -1;
    for (int k = 0; k < pasos; k++) {
        S.in[0][0] = voltaje(k);
        sfun_daqPic.salidas(&S, 0);
        if (k == 0) revisar(S.out[0][0] == 0.0, "posición 0 en el primer paso");
        sfun_daqPic.actualizar(&S, 0);
        if (k == 0) atrasos_paso1 = S.pwork[0] ? 0 : -1;
        if (k == 1) { sfun_daqPic.salidas(&S, 0); atrasos_paso1 = S.out[2][0]; }
    }
    sfun_daqPic.salidas(&S, 0);
    printf("  posición %.0f, errores %.0f, atrasos %.0f\n", S.out[0][0], S.out[1][0], S.out[2][0]);
    revisar(atrasos_paso1 == 0.0, "la pausa de inicialización no cuenta como atraso");
    revisar(S.out[1][0] == 0.0, "sin errores (la vigilancia de la pausa no cuenta)");
    sfun_daqPic.terminar(&S);
    revisar(S.pwork[0] == NULL, "mdlTerminate libera el estado");
}

static void prueba_bloques(int pasos, bool con_escritura)
{
    printf("daqPicInicio + %sdaqPicLeer, %d pasos a 1 kHz\n",
           con_escritura ? "daqPicEscribir + " : "", pasos);
    SimStruct Si, Se, Sl;
    preparar(sfun_daqPicInicio, Si, 0.001, 1.0);
    preparar(sfun_daqPicEscribir, Se, 0, 0);
    preparar(sfun_daqPicLeer, Sl, 0, 0);
    revisar(Si.nin == 0 && Si.nout == 1 && Se.nin == 2 && Se.nout == 0 &&
            Sl.nin == 1 && Sl.nout == 3, "puertos");
    revisar(Si.ts == 0.001 && Se.ts == INHERITED_SAMPLE_TIME && Sl.ts == INHERITED_SAMPLE_TIME,
            "periodos (Inicio fijo, los otros heredados)");

    /* Llaves inválidas: cero y una dirección válida sin la marca */
    double otra = 123.0;
    Se.in[0][0] = 0.0;
    sfun_daqPicEscribir.salidas(&Se, 0);
    revisar(Se.error != NULL, "Escribir rechaza una llave en 0");
    Sl.in[0][0] = (double)(uintptr_t)&otra;
    sfun_daqPicLeer.salidas(&Sl, 0);
    revisar(Sl.error != NULL, "Leer rechaza una llave que no viene de Inicio");
    Se.error = Sl.error = NULL;
    /* Una constante entera conectada por error no debe leerse como dirección */
    const double constantes[] = {1.0, 3.0, 1000.0, 0.5, -8.0, 1e300};
    bool rechazadas = true;
    for (double c : constantes) {
        Se.in[0][0] = c;
        sfun_daqPicEscribir.salidas(&Se, 0);
        rechazadas = rechazadas && Se.error != NULL;
        Se.error = NULL;
    }
    revisar(rechazadas, "Escribir rechaza constantes (1, 3, 1000, 0.5, -8, 1e300) sin leerlas");

    sfun_daqPicInicio.inicio(&Si);
    revisar(Si.error == NULL, Si.error ? Si.error : "mdlStart sin error");
    if (Si.error) { sfun_daqPicInicio.terminar(&Si); return; }
    usleep(300000);

    bool dac_ok = true;
    double atrasos_paso1 = -1;
    for (int k = 0; k < pasos; k++) {
        sfun_daqPicInicio.salidas(&Si, 0);
        Se.in[0][0] = Sl.in[0][0] = Si.out[0][0];
        Se.in[1][0] = voltaje(k);
        /* El orden entre Escribir y Leer no debe importar: se alterna */
        if (k % 2) {
            sfun_daqPicLeer.salidas(&Sl, 0);
            if (con_escritura) sfun_daqPicEscribir.salidas(&Se, 0);
        } else {
            if (con_escritura) sfun_daqPicEscribir.salidas(&Se, 0);
            sfun_daqPicLeer.salidas(&Sl, 0);
        }
        if (k == 0) revisar(Sl.out[0][0] == 0.0, "posición 0 en el primer paso");
        if (k == 2) atrasos_paso1 = Sl.out[2][0];
        const EstadoDaq *e = daq_estado(Si.out[0][0]);
        if (e == NULL || (con_escritura && e->dac != daq_voltaje_a_dac(voltaje(k))) ||
            e->salida != con_escritura) {
            dac_ok = false;
        }
        sfun_daqPicInicio.actualizar(&Si, 0);
    }
    sfun_daqPicLeer.salidas(&Sl, 0);
    printf("  posición %.0f, errores %.0f, atrasos %.0f\n", Sl.out[0][0], Sl.out[1][0], Sl.out[2][0]);
    revisar(Se.error == NULL && Sl.error == NULL, "Escribir y Leer aceptan la llave de Inicio");
    revisar(dac_ok, con_escritura ? "el voltaje de cada paso llega al estado antes de transferir"
                                  : "sin Escribir la salida queda deshabilitada");
    revisar(atrasos_paso1 == 0.0, "la pausa de inicialización no cuenta como atraso");
    revisar(Sl.out[1][0] == 0.0, "sin errores (la vigilancia de la pausa no cuenta)");
    const double llave = Si.out[0][0];
    sfun_daqPicEscribir.terminar(&Se);
    sfun_daqPicLeer.terminar(&Sl);
    sfun_daqPicInicio.terminar(&Si);
    revisar(Si.pwork[0] == NULL, "mdlTerminate de Inicio libera el estado");
    (void)llave;
}

int main(int argc, char **argv)
{
    const int pasos = argc > 1 ? atoi(argv[1]) : 10000;
    prueba_daqpic(pasos);
    prueba_bloques(pasos, true);
    prueba_bloques(pasos / 10, false);
    printf(fallas ? "\n%d FALLAS\n" : "\nTODO CORRECTO\n", fallas);
    return fallas != 0;
}

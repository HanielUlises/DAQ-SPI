/*
 * Hilo de adquisición del visor: ejecuta el ciclo de prueba de prueba_enlace
 * (herramientas/comun/enlace_prueba.h) sin depender del ritmo de la interfaz.
 *
 * Cada trama procesada se deja en un buffer circular de un productor y un
 * consumidor, del que la interfaz extrae todo lo nuevo en cada cuadro; si la
 * interfaz se atrasa más que la capacidad del buffer, las tramas sobrantes no
 * se grafican (se cuentan en descartadas()) pero sí entran en los contadores,
 * que el hilo publica completos tras cada trama.
 */
#ifndef ADQUISICION_H
#define ADQUISICION_H

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../comun/enlace_prueba.h"
#include "control.h"
#include "simulador.h"

enum Modo { MODO_DSPIC, MODO_LAZO, MODO_SIMULADO };

struct Configuracion {
    Modo modo = MODO_DSPIC;
    unsigned long tramas = 100000;      /* 0: sin límite de tramas */
    double duracion_s = 0.0;            /* 0: sin límite de tiempo */
    double periodo_us = 0.0;            /* 0: lo más rápido posible */
    unsigned long reloj = 1000000;      /* SCK en Hz */
    bool salida = false;                /* habilitar la salida desde la primera trama */
    bool registrar = false;
    std::string ruta_registro = "prueba.csv";
};

/* Una trama con las señales del modelo en ese paso */
struct Paso {
    Trama t;
    float r;                            /* referencia en cuentas; NAN sin lazo cerrado activo */
    int32_t y;                          /* posición relativa al origen, última válida */
};

/* Estado que el hilo publica para la interfaz */
struct Instantanea {
    EstadoPrueba e;
    unsigned long atrasos;              /* transferencias más largas que el periodo */
    double transcurrido_us;
};

class Adquisicion {
public:
    /* Controles que la interfaz cambia durante la prueba. La salida arranca
     * deshabilitada (0 V) salvo con Configuracion::salida; con ella
     * deshabilitada el campo del DAC varía al azar, como en prueba_enlace,
     * para ejercitar el protocolo. reset_encoder pone en cero el QEI del
     * dsPIC; cero sólo mueve el origen de la posición en la PC. */
    std::atomic<bool> salida{false};
    std::atomic<bool> reset_encoder{false};
    std::atomic<bool> cero{false};
    std::atomic<bool> sim_reinicio{false};
    std::atomic<double> sim_ber{0.0};

    Adquisicion();
    ~Adquisicion();

    /* Abre el canal y arranca el hilo. Si falla, el motivo queda en error() */
    bool iniciar(const Configuracion &conf);
    /* Pide detener la prueba y espera a que el hilo cierre el canal */
    void detener();
    /* Cierra el hilo si la prueba ya terminó sola; devuelve si sigue activa */
    bool activa();

    /* Cambia los parámetros del modelo. Si cambia la fuente, la referencia
     * o el tipo de lazo, el tiempo del generador vuelve a cero; las ganancias
     * se cambian sin reiniciar el PID. */
    void modelo(const Modelo &m);

    const std::string &error() const { return error_; }
    /* Resultado de fijar la afinidad y la prioridad del hilo */
    std::string aviso_tiempo_real();
    const Configuracion &configuracion() const { return conf_; }

    /* Copia hasta max pasos nuevos en destino y devuelve cuántos */
    size_t extraer(Paso *destino, size_t max);
    Instantanea instantanea();
    unsigned long descartadas() const { return descartadas_; }

private:
    static const size_t kCapacidad = 1u << 16;

    Configuracion conf_;
    std::string error_;
    std::thread hilo_;
    std::atomic<bool> detener_{false};
    std::atomic<bool> corriendo_{false};

    FT_HANDLE h_ = NULL;
    FILE *registro_ = NULL;
    Simulador sim_;

    std::vector<Paso> anillo_;
    std::atomic<size_t> escritura_{0};
    std::atomic<size_t> lectura_{0};
    std::atomic<unsigned long> descartadas_{0};

    std::mutex mutex_;
    Instantanea publicada_;
    std::string aviso_;
    Modelo modelo_;
    std::atomic<unsigned> version_modelo_{0};

    void tiempo_real();
    void ciclo();
};

/* Argumentos de prueba_enlace equivalentes a conf, para el registro CSV */
std::string comando_equivalente(const Configuracion &conf);

/* Descripción del modelo para el registro CSV */
std::string descripcion_modelo(const Modelo &m);

/* Voltaje que representa el campo del DAC de una trama de la PC (0 V si la
 * salida está deshabilitada) */
double voltaje_trama(const uint8_t *tx);

#endif /* ADQUISICION_H */

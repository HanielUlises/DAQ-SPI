/*
 * Modelo que ejecuta el visor en cada paso, en lugar de un modelo de
 * Simulink con daqPic:
 *
 *   lazo abierto:  fuente [V] -> saturación -> daqPic -> posición
 *   lazo cerrado:  referencia -> (+) -> PID -> saturación -> daqPic -> posición
 *                                 (-) <---------------------------------'
 *
 * La posición es relativa al origen (el inicio de la prueba o el último
 * "poner en cero") y se expresa en la unidad elegida. Como en daqPic, la
 * posición que usa el paso k es la que llegó en la transferencia k-1, por lo
 * que el retardo del lazo es de dos periodos.
 *
 * El PID es el Discrete PID Controller de Simulink en forma paralela:
 * integrador con Euler hacia adelante, derivada con filtro de coeficiente N
 * (Euler hacia atrás, estable para cualquier N Ts) y anti-windup por
 * sujeción: el integrador se detiene mientras la salida está saturada y el
 * error la empuja más hacia el límite.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <cmath>
#include <string>

/* Formas del generador de señal (como el Signal Generator de Simulink) */
enum Forma { FORMA_NADA, FORMA_ESCALON, FORMA_RAMPA, FORMA_SENO, FORMA_CUADRADA, FORMA_NUM };

static const char *const kFormas[FORMA_NUM] = {
    "nada (0)", "escalón", "rampa (triangular)", "senoidal", "cuadrada",
};

struct Generador {
    Forma forma = FORMA_RAMPA;
    double amplitud = 1.0;
    double frecuencia = 0.5;            /* Hz */
    double desplazamiento = 0.0;
    double retardo = 0.0;               /* s; antes vale el desplazamiento (step time) */

    /* Valor t segundos después de habilitar la salida, sin saturar */
    double valor(double t) const
    {
        if (forma == FORMA_NADA) return 0.0;
        if (t < retardo) return desplazamiento;
        t -= retardo;
        const double fase = frecuencia > 0.0 ? t * frecuencia - std::floor(t * frecuencia) : 0.0;
        double v = 0.0;
        switch (forma) {
        case FORMA_ESCALON:
            v = amplitud;
            break;
        case FORMA_RAMPA:
            v = amplitud * (fase < 0.5 ? -1.0 + 4.0 * fase : 3.0 - 4.0 * fase);
            break;
        case FORMA_SENO:
            v = amplitud * std::sin(2.0 * M_PI * fase);
            break;
        case FORMA_CUADRADA:
            v = fase < 0.5 ? amplitud : -amplitud;
            break;
        default:
            break;
        }
        return v + desplazamiento;
    }

    bool operator==(const Generador &o) const
    {
        return forma == o.forma && amplitud == o.amplitud && frecuencia == o.frecuencia &&
               desplazamiento == o.desplazamiento && retardo == o.retardo;
    }
    bool operator!=(const Generador &o) const { return !(*this == o); }

    std::string descripcion(const char *unidad) const;
};

enum Unidad { UNIDAD_CUENTAS, UNIDAD_VUELTAS, UNIDAD_GRADOS, UNIDAD_RAD, UNIDAD_NUM };

static const char *const kUnidades[UNIDAD_NUM] = {"cuentas", "vueltas", "grados", "rad"};

struct Pid {
    double kp = 0.003;                  /* V por unidad de posición */
    double ki = 0.0;                    /* V / (unidad s) */
    double kd = 0.0;                    /* V s / unidad */
    double n = 100.0;                   /* coeficiente del filtro de la derivada, 1/s */
    bool d_medicion = false;            /* derivada sobre -y en lugar de e (sin patada) */
};

struct Modelo {
    bool cerrado = false;
    Generador fuente;                   /* lazo abierto, en V */
    Generador referencia;               /* lazo cerrado, en la unidad de posición */
    Pid pid;
    double u_min = -2.5, u_max = 2.5;   /* saturación, V */
    Unidad unidad = UNIDAD_CUENTAS;
    double cuentas_por_vuelta = 4096.0;

    Modelo()
    {
        referencia.forma = FORMA_CUADRADA;
        referencia.amplitud = 500.0;
        referencia.frecuencia = 0.5;
    }

    /* Cuentas del encoder por unidad de posición */
    double escala() const
    {
        switch (unidad) {
        case UNIDAD_VUELTAS: return cuentas_por_vuelta;
        case UNIDAD_GRADOS:  return cuentas_por_vuelta / 360.0;
        case UNIDAD_RAD:     return cuentas_por_vuelta / (2.0 * M_PI);
        default:             return 1.0;
        }
    }

    double saturar(double u) const { return u < u_min ? u_min : u > u_max ? u_max : u; }
};

/* Estado del PID; reiniciar() lo deja como al empezar la simulación */
class ControladorPid {
public:
    void reiniciar()
    {
        integral_ = 0.0;
        derivada_ = 0.0;
        primero_ = true;
    }

    /* Un paso: referencia r y medición y en la unidad de posición, h el
     * periodo desde el paso anterior en s. Devuelve el voltaje saturado. */
    double paso(const Modelo &m, double r, double y, double h)
    {
        const Pid &p = m.pid;
        const double e = r - y;
        const double x = p.d_medicion ? -y : e;
        if (primero_ || h <= 0.0) {
            derivada_ = 0.0;
        } else {
            derivada_ = (derivada_ + p.kd * p.n * (x - x_anterior_)) / (1.0 + p.n * h);
        }
        x_anterior_ = x;
        const double sin_saturar = p.kp * e + integral_ + derivada_;
        const double u = m.saturar(sin_saturar);
        /* Sujeción: no integrar si la salida está saturada y e la empuja más allá */
        const bool empuja = (sin_saturar > u && e > 0.0) || (sin_saturar < u && e < 0.0);
        if (h > 0.0 && !empuja) integral_ += p.ki * h * e;
        primero_ = false;
        return u;
    }

    double integral() const { return integral_; }

private:
    double integral_ = 0.0;
    double derivada_ = 0.0;
    double x_anterior_ = 0.0;
    bool primero_ = true;
};

#endif /* CONTROL_H */

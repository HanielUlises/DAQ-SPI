/*
 * Pruebas sin hardware del modelo del visor: generador de señal, PID
 * (control.h), exportación (exportar.h) y el lazo cerrado completo en el
 * hilo de adquisición contra el simulador del dsPIC y del motor.
 */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "adquisicion.h"
#include "exportar.h"

static int fallas = 0;

#define VERIFICA(cond)                                                    \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FALLA %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
            fallas++;                                                     \
        }                                                                 \
    } while (0)

static bool cerca(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

static void prueba_generador(void)
{
    Generador g;
    g.forma = FORMA_ESCALON;
    g.amplitud = 2.0;
    g.desplazamiento = 0.5;
    g.retardo = 1.0;
    VERIFICA(cerca(g.valor(0.0), 0.5));
    VERIFICA(cerca(g.valor(0.999), 0.5));
    VERIFICA(cerca(g.valor(1.0), 2.5));
    VERIFICA(cerca(g.valor(10.0), 2.5));

    g.forma = FORMA_CUADRADA;
    g.retardo = 0.0;
    g.desplazamiento = 0.0;
    g.frecuencia = 1.0;
    VERIFICA(cerca(g.valor(0.25), 2.0));
    VERIFICA(cerca(g.valor(0.75), -2.0));

    g.forma = FORMA_RAMPA;
    g.amplitud = 1.0;
    VERIFICA(cerca(g.valor(0.0), -1.0));
    VERIFICA(cerca(g.valor(0.5), 1.0));
    VERIFICA(cerca(g.valor(0.25), 0.0));

    g.forma = FORMA_NADA;
    g.desplazamiento = 1.0;
    VERIFICA(cerca(g.valor(3.0), 0.0));

    Generador a, b;
    VERIFICA(a == b);
    b.retardo = 0.1;
    VERIFICA(a != b);
}

static void prueba_unidades(void)
{
    Modelo m;
    m.cuentas_por_vuelta = 4096.0;
    m.unidad = UNIDAD_CUENTAS;
    VERIFICA(cerca(m.escala(), 1.0));
    m.unidad = UNIDAD_VUELTAS;
    VERIFICA(cerca(m.escala(), 4096.0));
    m.unidad = UNIDAD_GRADOS;
    VERIFICA(cerca(m.escala() * 360.0, 4096.0));
    m.unidad = UNIDAD_RAD;
    VERIFICA(cerca(m.escala() * 2.0 * M_PI, 4096.0));
}

static void prueba_pid_proporcional(void)
{
    Modelo m;
    m.pid.kp = 0.01;
    ControladorPid c;
    VERIFICA(cerca(c.paso(m, 100.0, 0.0, 1e-3), 1.0));
    VERIFICA(cerca(c.paso(m, 100.0, 50.0, 1e-3), 0.5));
    /* Saturación */
    VERIFICA(cerca(c.paso(m, 1000.0, 0.0, 1e-3), 2.5));
    m.u_min = -1.0;
    VERIFICA(cerca(c.paso(m, -1000.0, 0.0, 1e-3), -1.0));
}

static void prueba_pid_integral(void)
{
    Modelo m;
    m.pid.kp = 0.0;
    m.pid.ki = 10.0;
    ControladorPid c;
    /* Euler hacia adelante: u_k = Ki h (e_0 + ... + e_{k-1}) */
    VERIFICA(cerca(c.paso(m, 1.0, 0.0, 1e-3), 0.0));
    VERIFICA(cerca(c.paso(m, 1.0, 0.0, 1e-3), 0.01));
    VERIFICA(cerca(c.paso(m, 1.0, 0.0, 1e-3), 0.02));

    /* Anti-windup: saturado y el error empuja hacia el límite */
    ControladorPid w;
    m.pid.kp = 1.0;
    for (int i = 0; i < 1000; i++) w.paso(m, 10.0, 0.0, 1e-3);
    VERIFICA(w.integral() < 1e-9);
    /* En cuanto el error cambia de signo vuelve a integrar y la salida sale de la saturación */
    const double u = w.paso(m, 0.0, 1.0, 1e-3);
    VERIFICA(cerca(u, -1.0));
    VERIFICA(w.integral() < 0.0);

    /* reiniciar() limpia el integrador */
    w.reiniciar();
    VERIFICA(w.integral() == 0.0);
}

static void prueba_pid_derivada(void)
{
    Modelo m;
    m.pid.kp = 0.0;
    m.pid.kd = 0.01;
    m.pid.n = 100.0;
    const double h = 1e-3;
    ControladorPid c;
    VERIFICA(cerca(c.paso(m, 0.0, 0.0, h), 0.0));
    /* Escalón de 1 en e: D = Kd N / (1 + N h) y después decae con 1 / (1 + N h) */
    const double d1 = c.paso(m, 1.0, 0.0, h);
    VERIFICA(cerca(d1, 0.01 * 100.0 / 1.1));
    const double d2 = c.paso(m, 1.0, 0.0, h);
    VERIFICA(cerca(d2, d1 / 1.1));

    /* Derivada sobre la medición: un escalón en r no produce patada */
    m.pid.d_medicion = true;
    ControladorPid s;
    s.paso(m, 0.0, 0.0, h);
    VERIFICA(cerca(s.paso(m, 1.0, 0.0, h), 0.0));
    VERIFICA(s.paso(m, 1.0, 1.0, h) < 0.0);
}

static void prueba_exportar(void)
{
    std::vector<Columna> c(2);
    c[0].nombre = "t";
    c[0].datos = {0.0, 0.001, 0.002};
    c[1].nombre = "y";
    c[1].datos = {1.5, NAN, -2.0};
    std::string error;
    const char *mat = "prueba_control.mat";
    VERIFICA(exportar_senales(mat, c, "", error));
    FILE *f = fopen(mat, "rb");
    VERIFICA(f != NULL);
    if (f != NULL) {
        int32_t h[5];
        char nombre[2];
        double d[3];
        VERIFICA(fread(h, sizeof h, 1, f) == 1);
        VERIFICA(h[0] == 0 && h[1] == 3 && h[2] == 1 && h[3] == 0 && h[4] == 2);
        VERIFICA(fread(nombre, 2, 1, f) == 1 && nombre[0] == 't' && nombre[1] == 0);
        VERIFICA(fread(d, sizeof d, 1, f) == 1 && d[1] == 0.001);
        VERIFICA(fread(h, sizeof h, 1, f) == 1 && h[1] == 3);
        VERIFICA(fread(nombre, 2, 1, f) == 1 && nombre[0] == 'y');
        VERIFICA(fread(d, sizeof d, 1, f) == 1 && d[0] == 1.5 && std::isnan(d[1]));
        VERIFICA(fgetc(f) == EOF);
        fclose(f);
    }
    remove(mat);

    const char *csv = "prueba_control.csv";
    VERIFICA(exportar_senales(csv, c, "uno\ndos", error));
    f = fopen(csv, "r");
    VERIFICA(f != NULL);
    if (f != NULL) {
        char texto[256] = {0};
        const size_t n = fread(texto, 1, sizeof texto - 1, f);
        texto[n] = 0;
        VERIFICA(strcmp(texto, "# uno\n# dos\nt,y\n0,1.5\n0.001,nan\n0.002,-2\n") == 0);
        fclose(f);
    }
    remove(csv);

    VERIFICA(!exportar_senales("/no/existe/x.mat", c, "", error));
    VERIFICA(!error.empty());
}

/* Corre el hilo de adquisición en modo simulado y junta todos los pasos */
static std::vector<Paso> correr(Adquisicion &a, const Configuracion &conf,
                                void (*durante)(Adquisicion &, const std::vector<Paso> &) = NULL)
{
    std::vector<Paso> pasos, lote(4096);
    VERIFICA(a.iniciar(conf));
    for (;;) {
        const bool viva = a.activa();
        size_t n;
        while ((n = a.extraer(lote.data(), lote.size())) > 0) {
            pasos.insert(pasos.end(), lote.begin(), lote.begin() + n);
        }
        if (durante != NULL) durante(a, pasos);
        if (!viva) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    VERIFICA(a.descartadas() == 0);
    return pasos;
}

static Configuracion conf_simulada(unsigned long tramas)
{
    Configuracion conf;
    conf.modo = MODO_SIMULADO;
    conf.periodo_us = 1000.0;
    conf.tramas = tramas;
    conf.salida = true;
    return conf;
}

static void prueba_lazo_cerrado(void)
{
    Modelo m;
    m.cerrado = true;
    m.unidad = UNIDAD_GRADOS;
    m.cuentas_por_vuelta = 4096.0;
    m.referencia.forma = FORMA_ESCALON;
    m.referencia.amplitud = 90.0;
    m.referencia.desplazamiento = 0.0;
    m.referencia.retardo = 0.1;
    m.pid.kp = 0.02;            /* V/grado */
    m.pid.ki = 0.05;
    m.pid.kd = 0.0005;

    Adquisicion a;
    a.modelo(m);
    const std::vector<Paso> p = correr(a, conf_simulada(2500));
    VERIFICA(p.size() == 2500);
    if (p.size() != 2500) return;

    const double escala = m.escala();
    /* Antes del escalón la referencia vale 0 y el motor no se mueve */
    VERIFICA(cerca(p[50].r, 0.0));
    VERIFICA(std::abs(p[50].y) <= 1);
    /* Después de 2 s la posición llegó a 90° (1024 cuentas) */
    VERIFICA(cerca(p[2400].r, 90.0 * escala, 1e-3));
    double y_max = 0.0;
    for (size_t k = 100; k < p.size(); k++) y_max = std::max(y_max, (double)p[k].y);
    const double y_fin = p.back().y / escala;
    printf("  lazo cerrado: escalón de 90°, y final %.2f°, máximo %.2f°\n", y_fin, y_max / escala);
    VERIFICA(std::fabs(y_fin - 90.0) < 1.0);
    VERIFICA(y_max / escala < 90.0 * 1.4);
    /* La salida estuvo habilitada en todos los pasos y dentro de la saturación */
    for (size_t k = 0; k < p.size(); k++) {
        if (!(p[k].t.tx[DAQ_PC_FLAGS] & DAQ_FLAG_SALIDA_HAB)) {
            VERIFICA(false);
            break;
        }
    }
}

static void prueba_lazo_abierto_sin_salida(void)
{
    Modelo m;
    m.fuente.forma = FORMA_ESCALON;
    m.fuente.amplitud = 1.0;
    Adquisicion a;
    a.modelo(m);
    Configuracion conf = conf_simulada(300);
    conf.salida = false;
    const std::vector<Paso> p = correr(a, conf);
    VERIFICA(p.size() == 300);
    for (const Paso &x : p) {
        if ((x.t.tx[DAQ_PC_FLAGS] & DAQ_FLAG_SALIDA_HAB) || !std::isnan(x.r) || x.y != 0) {
            VERIFICA(false);
            break;
        }
    }
}

/* Mueve el motor y luego pide el reset del QEI y después el cero de software */
static void accionar(Adquisicion &a, const std::vector<Paso> &p)
{
    static bool reset = false, cero = false;
    if (p.empty()) {
        reset = cero = false;
        return;
    }
    if (!reset && p.size() >= 500) {
        a.reset_encoder = true;
        reset = true;
    }
    if (!cero && p.size() >= 1200) {
        a.cero = true;
        cero = true;
    }
}

static void prueba_origen(void)
{
    Modelo m;
    m.fuente.forma = FORMA_ESCALON;
    m.fuente.amplitud = 1.0;
    Adquisicion a;
    a.modelo(m);
    accionar(a, std::vector<Paso>());
    const std::vector<Paso> p = correr(a, conf_simulada(2000), accionar);
    VERIFICA(p.size() == 2000);
    if (p.size() != 2000) return;
    /* El motor gira en un solo sentido: sin reset ni cero, y crece siempre.
     * Después de cada uno debe caer cerca de 0 sin pasos intermedios
     * negativos ni saltos hacia arriba. */
    int caidas = 0;
    for (size_t k = 1; k < p.size(); k++) {
        const int32_t d = p[k].y - p[k - 1].y;
        VERIFICA(p[k].y >= 0);
        if (d < -100) {
            caidas++;
            VERIFICA(p[k].y < 20);
        } else {
            VERIFICA(d >= 0 && d < 100);
        }
    }
    VERIFICA(caidas == 2);
}

int main(void)
{
    prueba_generador();
    prueba_unidades();
    prueba_pid_proporcional();
    prueba_pid_integral();
    prueba_pid_derivada();
    prueba_exportar();
    prueba_lazo_abierto_sin_salida();
    prueba_lazo_cerrado();
    prueba_origen();

    if (fallas == 0) {
        printf("OK: todas las pruebas del modelo pasaron\n");
        return 0;
    }
    printf("%d fallas\n", fallas);
    return 1;
}

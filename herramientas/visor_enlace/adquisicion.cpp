#include "adquisicion.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#ifndef _WIN32
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif

std::string Generador::descripcion(const char *unidad) const
{
    char buf[200];
    if (forma == FORMA_NADA) return "nada (0)";
    if (forma == FORMA_ESCALON) {
        snprintf(buf, sizeof buf, "escalón de %g a %g %s en t = %g s", desplazamiento,
                 amplitud + desplazamiento, unidad, retardo);
    } else {
        snprintf(buf, sizeof buf, "%s, amplitud %g %s, frecuencia %g Hz, desplazamiento %g %s",
                 kFormas[forma], amplitud, unidad, frecuencia, desplazamiento, unidad);
        if (retardo > 0.0) {
            const size_t n = strlen(buf);
            snprintf(buf + n, sizeof buf - n, ", desde t = %g s", retardo);
        }
    }
    return buf;
}

std::string descripcion_modelo(const Modelo &m)
{
    char buf[400];
    if (!m.cerrado) {
        snprintf(buf, sizeof buf, "lazo abierto, fuente %s, saturación [%g, %g] V",
                 m.fuente.descripcion("V").c_str(), m.u_min, m.u_max);
    } else {
        snprintf(buf, sizeof buf,
                 "lazo cerrado, referencia %s (%g cuentas/vuelta), PID kp %g ki %g kd %g N %g%s, "
                 "saturación [%g, %g] V",
                 m.referencia.descripcion(kUnidades[m.unidad]).c_str(), m.cuentas_por_vuelta,
                 m.pid.kp, m.pid.ki, m.pid.kd, m.pid.n, m.pid.d_medicion ? " (D sobre y)" : "",
                 m.u_min, m.u_max);
    }
    return buf;
}

double voltaje_trama(const uint8_t *tx)
{
    if (!(tx[DAQ_PC_FLAGS] & DAQ_FLAG_SALIDA_HAB)) return 0.0;
    const unsigned dac = ((unsigned)tx[DAQ_PC_DAC_H] << 8) | tx[DAQ_PC_DAC_L];
    return dac / DAQ_DAC_ESCALA - DAQ_V_MAX;
}

Adquisicion::Adquisicion() : anillo_(kCapacidad)
{
    prueba_iniciar(&publicada_.e);
    publicada_.atrasos = 0;
    publicada_.transcurrido_us = 0.0;
}

Adquisicion::~Adquisicion()
{
    detener();
}

std::string comando_equivalente(const Configuracion &conf)
{
    char buf[160];
    std::string s = "visor_enlace";
    if (conf.tramas > 0) {
        snprintf(buf, sizeof buf, " -n %lu", conf.tramas);
        s += buf;
    }
    if (conf.duracion_s > 0.0) {
        snprintf(buf, sizeof buf, " -tf %g", conf.duracion_s);
        s += buf;
    }
    if (conf.periodo_us > 0.0) {
        snprintf(buf, sizeof buf, " -periodo %g", conf.periodo_us);
        s += buf;
    }
    snprintf(buf, sizeof buf, " -reloj %lu", conf.reloj);
    s += buf;
    if (conf.modo == MODO_LAZO) s += " -lazo";
    if (conf.modo == MODO_SIMULADO) s += " -simulado";
    return s;
}

bool Adquisicion::iniciar(const Configuracion &conf)
{
    detener();
    conf_ = conf;
    error_.clear();

    if (conf_.registrar) {
        registro_ = fopen(conf_.ruta_registro.c_str(), "w");
        if (registro_ == NULL) {
            error_ = "No se pudo crear " + conf_.ruta_registro + ".";
            return false;
        }
        std::string visor;
        {
            std::lock_guard<std::mutex> l(mutex_);
            visor = descripcion_modelo(modelo_);
        }
        visor += conf_.salida ? "; salida habilitada desde el inicio"
                              : "; la salida arranca deshabilitada y se habilita durante la prueba";
        visor += " (bit 0 del byte flags de mosi)";
        registro_encabezado(registro_, comando_equivalente(conf_).c_str(), visor.c_str());
    }

    if (conf_.modo == MODO_SIMULADO) {
        sim_ = Simulador();
    } else {
        char error[128];
        if (!enlace_abrir((DWORD)conf_.reloj, conf_.modo == MODO_LAZO, &h_, error, sizeof error)) {
            error_ = error;
            if (registro_ != NULL) fclose(registro_);
            registro_ = NULL;
            return false;
        }
    }

    salida = conf_.salida;
    reset_encoder = false;
    cero = false;
    sim_reinicio = false;
    escritura_ = 0;
    lectura_ = 0;
    descartadas_ = 0;
    {
        std::lock_guard<std::mutex> l(mutex_);
        prueba_iniciar(&publicada_.e);
        publicada_.atrasos = 0;
        publicada_.transcurrido_us = 0.0;
        aviso_.clear();
    }
    detener_ = false;
    corriendo_ = true;
    hilo_ = std::thread(&Adquisicion::ciclo, this);
    return true;
}

void Adquisicion::detener()
{
    detener_ = true;
    if (hilo_.joinable()) hilo_.join();
}

bool Adquisicion::activa()
{
    if (!corriendo_ && hilo_.joinable()) hilo_.join();
    return corriendo_;
}

size_t Adquisicion::extraer(Paso *destino, size_t max)
{
    const size_t r = lectura_.load(std::memory_order_relaxed);
    const size_t w = escritura_.load(std::memory_order_acquire);
    size_t n = w - r;
    if (n > max) n = max;
    for (size_t i = 0; i < n; i++) {
        destino[i] = anillo_[(r + i) & (kCapacidad - 1)];
    }
    lectura_.store(r + n, std::memory_order_release);
    return n;
}

void Adquisicion::modelo(const Modelo &m)
{
    std::lock_guard<std::mutex> l(mutex_);
    modelo_ = m;
    version_modelo_++;
}

std::string Adquisicion::aviso_tiempo_real()
{
    std::lock_guard<std::mutex> l(mutex_);
    return aviso_;
}

/* Fija el hilo al último núcleo y pide SCHED_FIFO; sin privilegios lo
 * segundo falla y el hilo sigue con la prioridad normal */
void Adquisicion::tiempo_real()
{
    std::string aviso;
#ifndef _WIN32
    char buf[160];
    const long nucleos = sysconf(_SC_NPROCESSORS_ONLN);
    if (nucleos > 1) {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET((int)(nucleos - 1), &cpus);
        if (pthread_setaffinity_np(pthread_self(), sizeof cpus, &cpus) == 0) {
            snprintf(buf, sizeof buf, "hilo fijo en el núcleo %ld", nucleos - 1);
        } else {
            snprintf(buf, sizeof buf, "no se pudo fijar el núcleo");
        }
        aviso = buf;
    }
    sched_param p;
    p.sched_priority = 50;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &p) == 0) {
        aviso += ", SCHED_FIFO 50";
    } else {
        aviso += "; SCHED_FIFO no disponible (requiere CAP_SYS_NICE o límite rtprio), prioridad normal";
    }
#else
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    aviso = "prioridad THREAD_PRIORITY_TIME_CRITICAL";
#endif
    std::lock_guard<std::mutex> l(mutex_);
    aviso_ = aviso;
}

Instantanea Adquisicion::instantanea()
{
    std::lock_guard<std::mutex> l(mutex_);
    return publicada_;
}

void Adquisicion::ciclo()
{
    const bool lazo = conf_.modo == MODO_LAZO;
    const bool simulado = conf_.modo == MODO_SIMULADO;
    Instantanea local;
    EstadoPrueba &e = local.e;
    prueba_iniciar(&e);
    local.atrasos = 0;

    tiempo_real();
    Modelo mod;
    ControladorPid pid;
    unsigned version = ~0u;
    bool salida_anterior = false;
    double t_habilitada = 0.0, t_anterior = 0.0;

    /* Posición: la última válida y el origen, en cuentas del QEI */
    int32_t crudo = 0, origen = 0;
    bool hay_origen = false;
    bool reset_pendiente = false;
    unsigned long k_reset = 0;

    const double inicio = reloj_us();
    double siguiente = inicio;

    for (unsigned long k = 0; (conf_.tramas == 0 || k < conf_.tramas) && !detener_; k++) {
        Paso p;
        Trama &t = p.t;
        const bool sal = salida;
        /* Tiempo de la prueba: k Ts con periodo fijo, de pared sin él */
        const double t_s = conf_.periodo_us > 0.0 ? k * conf_.periodo_us * 1e-6
                                                  : (reloj_us() - inicio) * 1e-6;
        if (conf_.duracion_s > 0.0 && t_s > conf_.duracion_s) break;
        if (version_modelo_ != version) {
            std::lock_guard<std::mutex> l(mutex_);
            const Modelo &nuevo = modelo_;
            if (nuevo.fuente != mod.fuente || nuevo.referencia != mod.referencia ||
                nuevo.cerrado != mod.cerrado || version == ~0u) {
                t_habilitada = t_s;
            }
            if (nuevo.cerrado != mod.cerrado) pid.reiniciar();
            mod = nuevo;
            version = version_modelo_;
        }
        if (sal && !salida_anterior) {
            t_habilitada = t_s;
            pid.reiniciar();
        }
        salida_anterior = sal;
        if (cero.exchange(false)) origen = crudo;
        p.y = (int32_t)((uint32_t)crudo - (uint32_t)origen);
        p.r = NAN;

        /* El modelo calcula el voltaje con la posición que llegó en la trama anterior */
        const double h = conf_.periodo_us > 0.0 ? conf_.periodo_us * 1e-6 : t_s - t_anterior;
        t_anterior = t_s;
        uint16_t dac;
        if (!sal) {
            dac = aleatorio();
        } else if (mod.cerrado) {
            const double escala = mod.escala();
            const double r = mod.referencia.valor(t_s - t_habilitada);
            p.r = (float)(r * escala);
            dac = daq_voltaje_a_dac(pid.paso(mod, r, p.y / escala, h));
        } else {
            dac = daq_voltaje_a_dac(mod.saturar(mod.fuente.valor(t_s - t_habilitada)));
        }
        uint8_t flags = sal ? DAQ_FLAG_SALIDA_HAB : 0u;
        if (reset_encoder.exchange(false)) {
            flags |= DAQ_FLAG_RESET_ENC;
            reset_pendiente = true;
            k_reset = k;
        }
        DWORD n = 0;
        FT_STATUS st;

        t.k = k;
        daq_armar_trama_pc(t.tx, (uint8_t)k, dac, flags);

        if (conf_.periodo_us > 0.0) {
            siguiente += conf_.periodo_us;
            while (reloj_us() < siguiente) {
            }
        }

        const double t0 = reloj_us();
        t.t_us = t0 - inicio;
        if (simulado) {
            if (sim_reinicio.exchange(false)) sim_.arrancar(DAQ_REINICIO_MCLR);
            sim_.ber = sim_ber;
            sim_.transferir(t.tx, t.rx, t.t_us);
            const double fin = t0 + sim_.duracion_us((double)conf_.reloj);
            while (reloj_us() < fin) {
            }
            st = FT_OK;
            n = DAQ_TRAMA_LEN;
        } else {
            st = SPI_ReadWrite(h_, t.rx, t.tx, DAQ_TRAMA_LEN, &n, kOpciones);
        }
        t.dt_us = reloj_us() - t0;
        prueba_procesar(&e, &t, st, n, lazo);
        if (conf_.periodo_us > 0.0 && t.dt_us > conf_.periodo_us) local.atrasos++;
        registrar(registro_, &t);

        /* La respuesta de la trama k + 1 ya refleja el reset enviado en la k */
        if (t.status >= 0) {
            crudo = t.posicion;
            if (reset_pendiente && k > k_reset) {
                reset_pendiente = false;
                origen = 0;
            }
            if (!hay_origen) {
                hay_origen = true;
                origen = crudo;
            }
        }

        const size_t w = escritura_.load(std::memory_order_relaxed);
        if (w - lectura_.load(std::memory_order_acquire) < kCapacidad) {
            anillo_[w & (kCapacidad - 1)] = p;
            escritura_.store(w + 1, std::memory_order_release);
        } else {
            descartadas_++;
        }

        /* Nunca se espera a la interfaz: si está leyendo, se publica en la
         * siguiente trama */
        local.transcurrido_us = reloj_us() - inicio;
        if (mutex_.try_lock()) {
            publicada_ = local;
            mutex_.unlock();
        }
    }
    local.transcurrido_us = reloj_us() - inicio;
    {
        std::lock_guard<std::mutex> l(mutex_);
        publicada_ = local;
    }

    if (!simulado) {
        enlace_cerrar(h_, lazo, (uint8_t)e.c.tramas);
        h_ = NULL;
    }
    if (registro_ != NULL) {
        fclose(registro_);
        registro_ = NULL;
    }
    salida = false;     /* enlace_cerrar dejó la salida en 0 V */
    corriendo_ = false;
}

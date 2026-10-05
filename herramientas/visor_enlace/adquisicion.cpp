#include "adquisicion.h"

#include <cmath>
#include <cstdio>
#ifndef _WIN32
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif

double Generador::voltaje(double t) const
{
    const double fase = frecuencia > 0.0 ? t * frecuencia - std::floor(t * frecuencia) : 0.0;
    double v = 0.0;
    switch (forma) {
    case FORMA_NADA:
        return 0.0;
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

std::string Generador::descripcion() const
{
    char buf[160];
    if (forma == FORMA_NADA) return "nada (0 V)";
    if (forma == FORMA_ESCALON) {
        snprintf(buf, sizeof buf, "escalón de %g V al habilitar la salida", amplitud + desplazamiento);
    } else {
        snprintf(buf, sizeof buf, "%s, amplitud %g V, frecuencia %g Hz, desplazamiento %g V",
                 kFormas[forma], amplitud, frecuencia, desplazamiento);
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
            visor = "señal " + generador_.descripcion();
        }
        visor += "; la salida arranca deshabilitada y se habilita durante la prueba"
                 " (bit 0 del byte flags de mosi)";
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

    salida = false;
    reset_encoder = false;
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

size_t Adquisicion::extraer(Trama *destino, size_t max)
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

void Adquisicion::generador(const Generador &g)
{
    std::lock_guard<std::mutex> l(mutex_);
    generador_ = g;
    version_generador_++;
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
    Generador gen;
    unsigned version = ~0u;
    bool salida_anterior = false;
    double t_habilitada = 0.0;

    const double inicio = reloj_us();
    double siguiente = inicio;

    for (unsigned long k = 0; (conf_.tramas == 0 || k < conf_.tramas) && !detener_; k++) {
        Trama t;
        const bool sal = salida;
        /* Tiempo de la prueba: k Ts con periodo fijo, de pared sin él */
        const double t_s = conf_.periodo_us > 0.0 ? k * conf_.periodo_us * 1e-6
                                                  : (reloj_us() - inicio) * 1e-6;
        if (version_generador_ != version) {
            std::lock_guard<std::mutex> l(mutex_);
            gen = generador_;
            version = version_generador_;
            t_habilitada = t_s;
        }
        if (sal && !salida_anterior) t_habilitada = t_s;
        salida_anterior = sal;
        const uint16_t dac = sal ? daq_voltaje_a_dac(gen.voltaje(t_s - t_habilitada)) : aleatorio();
        uint8_t flags = sal ? DAQ_FLAG_SALIDA_HAB : 0u;
        if (reset_encoder.exchange(false)) flags |= DAQ_FLAG_RESET_ENC;
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

        const size_t w = escritura_.load(std::memory_order_relaxed);
        if (w - lectura_.load(std::memory_order_acquire) < kCapacidad) {
            anillo_[w & (kCapacidad - 1)] = t;
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

/*
 * Visor en tiempo real de la prueba del enlace PC <-> dsPIC, al estilo del
 * Scope de Simulink. Ejecuta el mismo ciclo que prueba_enlace en un hilo
 * aparte (adquisicion.h) y muestra, con el eje de tiempo ligado:
 *   - el voltaje de salida que se envía al DAC,
 *   - la posición del encoder,
 *   - la duración de cada transferencia, con el periodo y las tramas con error.
 * Además lleva los contadores de prueba_enlace, un generador de señal para la
 * salida, disparo por error, el mapa de bits erróneos contra la respuesta
 * esperada (bits.h, misma lógica que herramientas/reporte) y el registro CSV
 * compatible con herramientas/reporte.
 *
 * El eje de tiempo es k Ts cuando hay periodo fijo y tiempo de pared cuando
 * las tramas se envían lo más rápido posible.
 *
 * Uso: visor_enlace [-n tramas] [-periodo us] [-reloj Hz] [-lazo | -simulado]
 *                   [-registro archivo.csv] [-iniciar] [-salir] [-captura archivo.ppm]
 * -iniciar arranca la prueba al abrir. -salir cierra el visor al terminar la
 * prueba, imprime el resumen y devuelve 0 sin errores, 1 con errores y 2 si no
 * se pudo abrir el dispositivo, como prueba_enlace. -captura guarda la ventana
 * en formato PPM al terminar la prueba (con -salir) o a los 3 s, y sale.
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "implot.h"

#include "adquisicion.h"
#include "bits.h"

/* ---- Colores del Scope -------------------------------------------------- */

static const ImVec4 kAmarillo(1.0f, 1.0f, 0.0f, 1.0f);
static const ImVec4 kMagenta(1.0f, 0.0f, 1.0f, 1.0f);
static const ImVec4 kCian(0.0f, 1.0f, 1.0f, 1.0f);
static const ImVec4 kRojo(1.0f, 0.25f, 0.25f, 1.0f);
static const ImVec4 kVerde(0.25f, 1.0f, 0.25f, 1.0f);
static const ImVec4 kNaranja(1.0f, 0.6f, 0.1f, 1.0f);
static const ImVec4 kGris(0.6f, 0.6f, 0.6f, 1.0f);
static const ImVec4 kBlanco(1.0f, 1.0f, 1.0f, 1.0f);

/* ---- Historial de muestras ---------------------------------------------- */

struct Muestra {
    double t;           /* s */
    float dt;           /* duración de la transferencia, us */
    float voltaje;      /* V */
    int32_t posicion;   /* cuentas; se mantiene la última válida */
};

/* Buffer circular con las últimas muestras, en orden de tiempo */
class Historial {
public:
    explicit Historial(size_t capacidad) : datos_(capacidad) {}
    void limpiar() { inicio_ = n_ = 0; }
    size_t size() const { return n_; }
    const Muestra &operator[](size_t i) const { return datos_[(inicio_ + i) % datos_.size()]; }

    void agregar(const Muestra &m)
    {
        if (n_ < datos_.size()) {
            datos_[(inicio_ + n_++) % datos_.size()] = m;
        } else {
            datos_[inicio_] = m;
            inicio_ = (inicio_ + 1) % datos_.size();
        }
    }

    /* Primer índice con t >= x */
    size_t buscar(double x) const
    {
        size_t a = 0, b = n_;
        while (a < b) {
            size_t m = (a + b) / 2;
            if ((*this)[m].t < x) a = m + 1; else b = m;
        }
        return a;
    }

private:
    std::vector<Muestra> datos_;
    size_t inicio_ = 0, n_ = 0;
};

/* Reduce las muestras [i0, i1) a mínimo y máximo por columna, para que los
 * picos de una serie larga no desaparezcan al graficarla */
template <class F>
static void decimar(const Historial &h, size_t i0, size_t i1, int columnas, F valor,
                    std::vector<double> &xs, std::vector<double> &ys)
{
    xs.clear();
    ys.clear();
    const size_t n = i1 - i0;
    if (columnas < 1) columnas = 1;
    if (n <= (size_t)columnas * 2) {
        for (size_t i = i0; i < i1; i++) {
            xs.push_back(h[i].t);
            ys.push_back(valor(h[i]));
        }
        return;
    }
    const double paso = (double)n / columnas;
    for (int c = 0; c < columnas; c++) {
        size_t a = i0 + (size_t)(c * paso);
        size_t b = std::min(i1, i0 + (size_t)((c + 1) * paso));
        if (a >= b) continue;
        size_t imin = a, imax = a;
        double vmin = valor(h[a]), vmax = vmin;
        for (size_t i = a + 1; i < b; i++) {
            const double v = valor(h[i]);
            if (v < vmin) { vmin = v; imin = i; }
            if (v > vmax) { vmax = v; imax = i; }
        }
        const size_t p = std::min(imin, imax), q = std::max(imin, imax);
        xs.push_back(h[p].t);
        ys.push_back(valor(h[p]));
        if (q != p) {
            xs.push_back(h[q].t);
            ys.push_back(valor(h[q]));
        }
    }
}

/* ---- Tramas con error, por categoría ------------------------------------ */

enum Categoria {
    CAT_USB, CAT_ECO, CAT_INICIO, CAT_CRC, CAT_SEQ,
    CAT_PIC_CRC, CAT_PIC_INICIO, CAT_PIC_LONGITUD, CAT_VIGILANCIA, CAT_REINICIO, CAT_NUM
};

static const char *const kCategorias[CAT_NUM] = {
    "USB / longitud", "eco distinto", "encabezado", "CRC", "secuencia",
    "CRC (dsPIC)", "encabezado (dsPIC)", "longitud (dsPIC)", "vigilancia", "reinicio",
};

static const ImVec4 kColorCategoria[CAT_NUM] = {
    kRojo, kRojo, kMagenta, kRojo, kCian, kNaranja, kMagenta, kVerde, kBlanco, kBlanco,
};

static const ImPlotMarker kMarcaCategoria[CAT_NUM] = {
    ImPlotMarker_Square, ImPlotMarker_Cross, ImPlotMarker_Diamond, ImPlotMarker_Circle,
    ImPlotMarker_Up, ImPlotMarker_Circle, ImPlotMarker_Diamond, ImPlotMarker_Square,
    ImPlotMarker_Up, ImPlotMarker_Cross,
};

/* Categorías de error de una trama, con el criterio de prueba_procesar */
static unsigned categorias(const Trama &t)
{
    unsigned c = 0;
    switch (t.resultado) {
    case RES_USB:    c |= 1u << CAT_USB; break;
    case RES_ECO:    c |= 1u << CAT_ECO; break;
    case RES_INICIO: c |= 1u << CAT_INICIO; break;
    case RES_CRC:    c |= 1u << CAT_CRC; break;
    case RES_SEQ:    c |= 1u << CAT_SEQ; break;
    default: break;
    }
    if (t.status >= 0 && (t.resultado == RES_OK || t.resultado == RES_SEQ)) {
        const uint8_t s = status_contable(&t);
        if (s & DAQ_STATUS_ERR_CRC) c |= 1u << CAT_PIC_CRC;
        if (s & DAQ_STATUS_ERR_INICIO) c |= 1u << CAT_PIC_INICIO;
        if (s & DAQ_STATUS_ERR_LONGITUD) c |= 1u << CAT_PIC_LONGITUD;
        if (s & DAQ_STATUS_VIGILANCIA) c |= 1u << CAT_VIGILANCIA;
        if (s & DAQ_STATUS_REINICIO) c |= 1u << CAT_REINICIO;
    }
    return c;
}

struct Marcas {
    std::vector<double> t, dt;
};

static std::string texto_status(int status)
{
    if (status < 0) return "-";
    char buf[16];
    snprintf(buf, sizeof buf, "%02X", status);
    std::string s = buf;
    if (status & DAQ_STATUS_ERR_CRC) s += " CRC";
    if (status & DAQ_STATUS_ERR_INICIO) s += " ENC";
    if (status & DAQ_STATUS_ERR_LONGITUD) s += " LON";
    if (status & DAQ_STATUS_VIGILANCIA) s += " VIG";
    if (status & DAQ_STATUS_REINICIO) {
        s += " reinicio: ";
        s += kCausasReinicio[(status & DAQ_STATUS_REINICIO) >> DAQ_STATUS_REINICIO_POS];
    }
    return s;
}

/* ---- Estado de la interfaz ---------------------------------------------- */

/* Histograma fino de la duración de las transferencias de toda la prueba */
static const double kAnchoFino = 10.0;
static const int kCubetasFinas = 400;

enum EstadoDisparo { DISPARO_ARMADO, DISPARO_ESPERANDO, DISPARO_DISPARADO };

struct Visor {
    Adquisicion adq;
    Configuracion conf;
    Generador gen;
    bool iniciada = false;      /* hubo al menos una prueba desde que se abrió */
    std::string mensaje;

    /* Datos de la prueba en curso o de la última */
    Historial hist{1u << 21};
    Marcas marcas[CAT_NUM];
    double fino[kCubetasFinas] = {};
    AnalisisBits bits{500};
    Trama ultima;
    bool hay_ultima = false;
    int32_t ultima_posicion = 0;
    double periodo_us = 0.0;
    bool lazo = false;
    Instantanea inst;
    std::vector<Trama> lote = std::vector<Trama>(1u << 14);

    /* Vista */
    double lapso = 5.0;
    bool seguir = true;
    double x_min = 0.0, x_max = 5.0;
    bool autoescala = true;
    bool escalar_y = false;
    bool cursores = false;
    double cursor[2] = {0.0, 0.0};
    bool disparo = false;
    EstadoDisparo estado_disparo = DISPARO_ARMADO;
    double t_disparo = 0.0;
    std::string causa_disparo;
    bool histograma_log = true;
    bool dt_log = false;
    float fraccion_displays = 0.62f;    /* alto de los displays sobre el panel derecho */
    float filas_displays[3] = {1.0f, 1.0f, 1.2f};
    bool terminada = false;             /* la prueba terminó y ya se extrajeron todas sus tramas */

    /* Tiempo de ciclo de la interfaz */
    double t_cuadro = 0.0, cuadro_ms = 0.0, cuadro_max_ms = 0.0, trabajo_ms = 0.0;
    double cuadro_max_acum = 0.0, t_ventana_max = 0.0;

    ImFont *mono = NULL;
};

static void limpiar_datos(Visor &v)
{
    v.hist.limpiar();
    for (int c = 0; c < CAT_NUM; c++) {
        v.marcas[c].t.clear();
        v.marcas[c].dt.clear();
    }
    memset(v.fino, 0, sizeof v.fino);
    v.bits.reiniciar(v.lazo);
    v.hay_ultima = false;
    v.ultima_posicion = 0;
    v.estado_disparo = DISPARO_ARMADO;
    v.terminada = false;
    v.x_min = 0.0;
    v.x_max = v.lapso;
}

static void iniciar(Visor &v)
{
    v.periodo_us = v.conf.periodo_us;
    v.lazo = v.conf.modo == MODO_LAZO;
    v.adq.generador(v.gen);
    if (v.adq.iniciar(v.conf)) {
        limpiar_datos(v);
        v.iniciada = true;
        v.seguir = true;
        v.mensaje.clear();
    } else {
        v.mensaje = v.adq.error();
    }
}

static void disparar_si_corresponde(Visor &v, const Trama &t, double ts, unsigned cats)
{
    if (!v.disparo || v.estado_disparo != DISPARO_ARMADO || cats == 0) return;
    v.estado_disparo = DISPARO_ESPERANDO;
    v.t_disparo = ts;
    char buf[96];
    for (int c = 0; c < CAT_NUM; c++) {
        if (cats & (1u << c)) {
            snprintf(buf, sizeof buf, "%s en la trama %lu", kCategorias[c], t.k);
            v.causa_disparo = buf;
            break;
        }
    }
}

static void procesar(Visor &v, const Trama &t)
{
    const double ts = v.periodo_us > 0.0 ? t.k * v.periodo_us * 1e-6 : t.t_us * 1e-6;
    if (t.status >= 0) v.ultima_posicion = t.posicion;
    Muestra m;
    m.t = ts;
    m.dt = (float)t.dt_us;
    m.voltaje = (float)voltaje_trama(t.tx);
    m.posicion = v.ultima_posicion;
    v.hist.agregar(m);

    int cubeta = (int)(t.dt_us / kAnchoFino);
    v.fino[cubeta < kCubetasFinas ? cubeta : kCubetasFinas - 1]++;

    const unsigned cats = categorias(t);
    for (int c = 0; c < CAT_NUM; c++) {
        if ((cats & (1u << c)) && v.marcas[c].t.size() < 1000000) {
            v.marcas[c].t.push_back(ts);
            v.marcas[c].dt.push_back(t.dt_us);
        }
    }
    disparar_si_corresponde(v, t, ts, cats);
    v.bits.agregar(t);
    v.ultima = t;
    v.hay_ultima = true;
}

/* Extrae las tramas nuevas y decide la ventana de tiempo */
static void actualizar(Visor &v)
{
    /* Si el hilo ya había terminado antes de extraer, no quedan tramas pendientes */
    const bool viva = v.adq.activa();
    size_t n;
    while ((n = v.adq.extraer(v.lote.data(), v.lote.size())) > 0) {
        for (size_t i = 0; i < n; i++) procesar(v, v.lote[i]);
    }
    v.inst = v.adq.instantanea();
    v.terminada = v.iniciada && !viva;

    const double t_ult = v.hist.size() > 0 ? v.hist[v.hist.size() - 1].t : 0.0;
    if (v.disparo && v.estado_disparo == DISPARO_ESPERANDO &&
        (t_ult >= v.t_disparo + 0.8 * v.lapso || v.terminada)) {
        v.estado_disparo = DISPARO_DISPARADO;
        v.x_min = v.t_disparo - 0.2 * v.lapso;
        v.x_max = v.t_disparo + 0.8 * v.lapso;
    }
    const bool congelada = v.disparo && v.estado_disparo == DISPARO_DISPARADO;
    if (v.seguir && !congelada) {
        v.x_max = std::max(t_ult, v.lapso);
        v.x_min = v.x_max - v.lapso;
    }
}

/* ---- Paneles ------------------------------------------------------------ */

static void ayuda(const char *texto)
{
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
        ImGui::TextUnformatted(texto);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

static void fila_contador(const char *nombre, unsigned long valor, bool es_error = true)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(nombre);
    ImGui::TableNextColumn();
    if (es_error && valor > 0) {
        ImGui::TextColored(kRojo, "%lu", valor);
    } else {
        ImGui::Text("%lu", valor);
    }
}

static void panel_prueba(Visor &v)
{
    const bool activa = v.adq.activa();
    ImGui::SeparatorText("Prueba");
    ImGui::BeginDisabled(activa);
    int modo = (int)v.conf.modo;
    ImGui::RadioButton("dsPIC", &modo, MODO_DSPIC);
    ImGui::SameLine();
#ifndef _WIN32
    ImGui::RadioButton("lazo FT2232H", &modo, MODO_LAZO);
    ImGui::SameLine();
#endif
    ImGui::RadioButton("simulado", &modo, MODO_SIMULADO);
    ayuda("dsPIC: tarjeta completa. Lazo: MOSI unido a MISO dentro del FT2232H, sin dsPIC. "
          "Simulado: modelo del dsPIC y del motor, sin hardware.");
    v.conf.modo = (Modo)modo;

    bool sin_limite = v.conf.tramas == 0;
    int tramas = (int)(sin_limite ? 100000 : v.conf.tramas);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    ImGui::BeginDisabled(sin_limite);
    if (ImGui::InputInt("tramas", &tramas, 10000, 100000)) v.conf.tramas = (unsigned long)std::max(1, tramas);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Checkbox("sin límite", &sin_limite)) v.conf.tramas = sin_limite ? 0 : 100000;

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    ImGui::InputDouble("periodo [us]", &v.conf.periodo_us, 100.0, 1000.0, "%.0f");
    if (v.conf.periodo_us < 0.0) v.conf.periodo_us = 0.0;
    ayuda("0: lo más rápido posible. Con periodo fijo el eje de tiempo es k Ts.");
    int reloj = (int)v.conf.reloj;
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    if (ImGui::InputInt("SCK [Hz]", &reloj, 100000, 1000000)) {
        v.conf.reloj = (unsigned long)std::max(100000, std::min(30000000, reloj));
    }
    ImGui::Checkbox("registro CSV", &v.conf.registrar);
    ImGui::SameLine();
    char ruta[256];
    snprintf(ruta, sizeof ruta, "%s", v.conf.ruta_registro.c_str());
    ImGui::SetNextItemWidth(-1);
    ImGui::BeginDisabled(!v.conf.registrar);
    if (ImGui::InputText("##ruta", ruta, sizeof ruta)) v.conf.ruta_registro = ruta;
    ImGui::EndDisabled();
    ImGui::EndDisabled();

    const ImVec2 boton(-1, ImGui::GetFrameHeight() * 1.4f);
    if (!activa) {
        if (ImGui::Button("Iniciar", boton)) iniciar(v);
    } else {
        if (ImGui::Button("Detener", boton)) v.adq.detener();
    }
    if (!v.mensaje.empty()) ImGui::TextColored(kRojo, "%s", v.mensaje.c_str());
}

static void panel_estado(Visor &v)
{
    const Contadores &c = v.inst.e.c;
    const unsigned long errores = prueba_errores(&c);
    ImGui::SeparatorText("Estado");
    if (!v.iniciada) {
        ImGui::TextDisabled("Sin prueba.");
    } else if (errores == 0) {
        ImGui::TextColored(kVerde, "SIN ERRORES");
    } else {
        ImGui::TextColored(kRojo, "CON ERRORES");
    }
    if (v.iniciada) {
        ImGui::SameLine();
        ImGui::TextDisabled(v.adq.activa() ? "(en curso)" : "(terminada)");
    }
    const double t = v.inst.transcurrido_us * 1e-6;
    ImGui::Text("%lu tramas en %.1f s (%.0f tramas/s)", c.tramas, t, t > 0.0 ? c.tramas / t : 0.0);
    if (v.iniciada && v.adq.configuracion().tramas > 0) {
        ImGui::ProgressBar((float)c.tramas / (float)v.adq.configuracion().tramas, ImVec2(-1, 0));
    }
    if (ImGui::BeginTable("totales", 2, ImGuiTableFlags_SizingStretchProp)) {
        fila_contador("Errores", errores);
        fila_contador("Atrasos (dt > periodo)", v.inst.atrasos);
        ImGui::EndTable();
    }
    if (v.adq.descartadas() > 0) {
        ImGui::TextColored(kNaranja, "%lu tramas sin graficar (la interfaz se atrasó)",
                           v.adq.descartadas());
    }
    const std::string aviso = v.adq.aviso_tiempo_real();
    if (!aviso.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Adquisición: %s", aviso.c_str());
        ImGui::PopTextWrapPos();
    }
    ImGui::TextColored(v.cuadro_max_ms > 50.0 ? kNaranja : kGris,
                       "Ciclo GUI: %.1f ms (máx %.0f ms, trabajo %.1f ms)",
                       v.cuadro_ms, v.cuadro_max_ms, v.trabajo_ms);
    ayuda("Duración de cada cuadro de la interfaz. La interfaz corre en otro hilo, pero la carga "
          "de la PC ensucia la cola de dt: para pruebas de aceptación conviene prueba_enlace.");
}

static void panel_salida(Visor &v)
{
    const bool activa = v.adq.activa();
    ImGui::SeparatorText("Salida analógica");
    const bool sal = v.adq.salida;
    const ImVec2 boton(-1, ImGui::GetFrameHeight() * 1.4f);
    ImGui::BeginDisabled(!activa);
    if (sal) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.1f, 0.1f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
        if (ImGui::Button("SALIDA HABILITADA (clic para 0 V)", boton)) v.adq.salida = false;
        ImGui::PopStyleColor(2);
    } else {
        if (ImGui::Button("Habilitar salida (el motor se mueve)", boton)) v.adq.salida = true;
    }
    ImGui::EndDisabled();
    if (!activa) ImGui::TextDisabled("La salida se habilita durante la prueba y arranca en 0 V.");

    bool cambio = false;
    int forma = (int)v.gen.forma;
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
    if (ImGui::Combo("señal", &forma, kFormas, FORMA_NUM)) {
        v.gen.forma = (Forma)forma;
        cambio = true;
    }
    ImGui::BeginDisabled(v.gen.forma == FORMA_NADA);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    cambio |= ImGui::SliderScalar("amplitud [V]", ImGuiDataType_Double, &v.gen.amplitud,
                                  &(const double &)0.0, &(const double &)DAQ_V_MAX, "%.2f");
    ImGui::BeginDisabled(v.gen.forma == FORMA_ESCALON);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    static const double f_min = 0.01, f_max = 100.0, d_min = -DAQ_V_MAX, d_max = DAQ_V_MAX;
    cambio |= ImGui::SliderScalar("frecuencia [Hz]", ImGuiDataType_Double, &v.gen.frecuencia,
                                  &f_min, &f_max, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::EndDisabled();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    cambio |= ImGui::SliderScalar("desplazamiento [V]", ImGuiDataType_Double,
                                  &v.gen.desplazamiento, &d_min, &d_max, "%.2f");
    ImGui::EndDisabled();
    if (cambio) v.adq.generador(v.gen);
    const double alto = v.gen.forma == FORMA_NADA ? 0.0 : v.gen.desplazamiento + v.gen.amplitud;
    const double bajo = v.gen.forma == FORMA_NADA ? 0.0
                        : v.gen.forma == FORMA_ESCALON ? alto
                        : v.gen.desplazamiento - v.gen.amplitud;
    if (alto > DAQ_V_MAX || bajo < -DAQ_V_MAX) {
        ImGui::TextColored(kNaranja, "Se satura a ±%.1f V", DAQ_V_MAX);
    }

    ImGui::BeginDisabled(!activa);
    if (ImGui::Button("Reset del encoder")) v.adq.reset_encoder = true;
    ImGui::EndDisabled();

    if (v.conf.modo == MODO_SIMULADO) {
        ImGui::SeparatorText("Simulador");
        float ber = (float)v.adq.sim_ber.load();
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
        if (ImGui::SliderFloat("errores por bit", &ber, 0.0f, 0.01f, "%.1e",
                               ImGuiSliderFlags_Logarithmic)) {
            v.adq.sim_ber = ber;
        }
        ImGui::BeginDisabled(!activa);
        if (ImGui::Button("Simular reinicio (MCLR)")) v.adq.sim_reinicio = true;
        ImGui::EndDisabled();
    }
}

static void panel_contadores(Visor &v)
{
    const Contadores &c = v.inst.e.c;
    ImGui::SeparatorText("Contadores");
    if (!ImGui::BeginTable("contadores", 2, ImGuiTableFlags_SizingStretchProp)) return;
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("Errores en la PC");
    fila_contador("  USB / longitud", c.err_usb);
    if (v.lazo) fila_contador("  eco distinto del envío", c.err_eco);
    fila_contador("  encabezado", c.err_inicio);
    fila_contador("  CRC", c.err_crc);
    fila_contador("  número de secuencia", c.err_seq);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("Reportados por el dsPIC");
    fila_contador("  CRC", c.pic_crc);
    fila_contador("  encabezado", c.pic_inicio);
    fila_contador("  longitud", c.pic_longitud);
    fila_contador("  vigilancia", c.pic_vigilancia);
    fila_contador("  reinicios", prueba_reinicios(&c));
    for (int i = 1; i < 16; i++) {
        if (c.pic_reinicio[i] > 0) {
            char nombre[64];
            snprintf(nombre, sizeof nombre, "    %s", kCausasReinicio[i]);
            fila_contador(nombre, c.pic_reinicio[i]);
        }
    }
    ImGui::EndTable();
    if (c.tramas > 0) {
        ImGui::Text("dt: mín %.0f, media %.0f, máx %.0f us", v.inst.e.t_min,
                    v.inst.e.t_suma / c.tramas, v.inst.e.t_max);
    }
}

static void panel_vista(Visor &v)
{
    ImGui::SeparatorText("Vista");
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    if (ImGui::InputDouble("lapso [s]", &v.lapso, 1.0, 5.0, "%.2f")) {
        v.lapso = std::max(0.01, v.lapso);
    }
    ayuda("Time span: ancho de la ventana de tiempo.");
    const bool congelada = v.disparo && v.estado_disparo == DISPARO_DISPARADO;
    if (ImGui::Button(v.seguir && !congelada ? "Pausa" : "Seguir")) {
        if (congelada) {
            v.estado_disparo = DISPARO_ARMADO;
            v.seguir = true;
        } else {
            v.seguir = !v.seguir;
        }
    }
    ayuda("Pausa congela la vista sin detener la prueba; en pausa se puede desplazar y "
          "acercar con el ratón (doble clic: ajustar).");
    ImGui::SameLine();
    if (ImGui::Button("Escalar Y")) v.escalar_y = true;
    ImGui::SameLine();
    ImGui::Checkbox("autoescala", &v.autoescala);
    ImGui::Checkbox("dt en escala log", &v.dt_log);
    ImGui::SameLine();
    if (ImGui::Checkbox("cursores", &v.cursores) && v.cursores) {
        v.cursor[0] = v.x_min + 0.33 * (v.x_max - v.x_min);
        v.cursor[1] = v.x_min + 0.66 * (v.x_max - v.x_min);
    }

    ImGui::Checkbox("disparo por error", &v.disparo);
    ayuda("Congela la ventana en la primera trama con error (CRC, encabezado, secuencia, USB, "
          "banderas del dsPIC o reinicio), con 20 % de pre-disparo.");
    if (v.disparo) {
        ImGui::SameLine();
        if (v.estado_disparo == DISPARO_ARMADO) {
            ImGui::TextColored(kVerde, "armado");
        } else {
            ImGui::TextColored(kNaranja, v.estado_disparo == DISPARO_ESPERANDO ? "disparando" : "disparado");
            ImGui::SameLine();
            if (ImGui::SmallButton("Rearmar")) {
                v.estado_disparo = DISPARO_ARMADO;
                v.seguir = true;
            }
            ImGui::TextWrapped("%s", v.causa_disparo.c_str());
        }
    }
}

/* ---- Displays ----------------------------------------------------------- */

static void ejes(Visor &v, const char *etiqueta_y, double y_min, double y_max)
{
    ImPlot::SetupAxes(NULL, etiqueta_y, ImPlotAxisFlags_NoLabel,
                      v.autoescala ? ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit : 0);
    ImPlot::SetupAxisLinks(ImAxis_X1, &v.x_min, &v.x_max);
    ImPlot::SetupAxisLimits(ImAxis_Y1, y_min, y_max, ImPlotCond_Once);
    if (v.escalar_y) ImPlot::SetNextAxisToFit(ImAxis_Y1);
}

static void dibujar_cursores(Visor &v)
{
    if (!v.cursores) return;
    ImPlot::DragLineX(0, &v.cursor[0], kCian, 1.5f);
    ImPlot::DragLineX(1, &v.cursor[1], kMagenta, 1.5f);
}

static void displays(Visor &v, float alto)
{
    const size_t i0 = v.hist.buscar(v.x_min);
    const size_t i1 = std::min(v.hist.size(), v.hist.buscar(v.x_max) + 1);
    const size_t j0 = i0 > 0 ? i0 - 1 : 0;   /* una muestra antes para no cortar la línea */
    const int columnas = (int)ImGui::GetContentRegionAvail().x;
    static std::vector<double> xs, ys;
    const ImPlotSpec traza(ImPlotProp_LineColor, kAmarillo, ImPlotProp_LineWeight, 1.5f);

    if (!ImPlot::BeginSubplots("##displays", 3, 1, ImVec2(-1, alto), ImPlotSubplotFlags_NoTitle,
                               v.filas_displays)) {
        return;
    }
    if (ImPlot::BeginPlot("Voltaje de salida [V]", ImVec2(-1, 0), ImPlotFlags_NoLegend)) {
        ejes(v, "V", -2.75, 2.75);
        decimar(v.hist, j0, i1, columnas, [](const Muestra &m) { return (double)m.voltaje; }, xs, ys);
        ImPlot::PlotLine("voltaje", xs.data(), ys.data(), (int)xs.size(), traza);
        dibujar_cursores(v);
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Posición del encoder [cuentas]", ImVec2(-1, 0), ImPlotFlags_NoLegend)) {
        ejes(v, "cuentas", -100.0, 100.0);
        decimar(v.hist, j0, i1, columnas, [](const Muestra &m) { return (double)m.posicion; }, xs, ys);
        ImPlot::PlotLine("posición", xs.data(), ys.data(), (int)xs.size(), traza);
        dibujar_cursores(v);
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Duración de cada transferencia [us]", ImVec2(-1, 0))) {
        ejes(v, "us", v.dt_log ? 50.0 : 0.0, 500.0);
        if (v.dt_log) ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::SetupLegend(ImPlotLocation_North, ImPlotLegendFlags_Outside | ImPlotLegendFlags_Horizontal);
        decimar(v.hist, j0, i1, columnas, [](const Muestra &m) { return (double)m.dt; }, xs, ys);
        ImPlot::PlotLine("dt", xs.data(), ys.data(), (int)xs.size(), traza);
        if (v.periodo_us > 0.0) {
            ImPlot::PlotInfLines("periodo", &v.periodo_us, 1,
                                 ImPlotSpec(ImPlotProp_LineColor, kGris,
                                            ImPlotProp_Flags, ImPlotInfLinesFlags_Horizontal));
        }
        for (int c = 0; c < CAT_NUM; c++) {
            const Marcas &m = v.marcas[c];
            const size_t a = std::lower_bound(m.t.begin(), m.t.end(), v.x_min) - m.t.begin();
            const size_t b = std::upper_bound(m.t.begin(), m.t.end(), v.x_max) - m.t.begin();
            if (b <= a) continue;
            ImPlot::PlotScatter(kCategorias[c], &m.t[a], &m.dt[a], (int)(b - a),
                                ImPlotSpec(ImPlotProp_Marker, kMarcaCategoria[c],
                                           ImPlotProp_MarkerSize, 5.0f,
                                           ImPlotProp_MarkerFillColor, kColorCategoria[c],
                                           ImPlotProp_MarkerLineColor, kColorCategoria[c],
                                           ImPlotProp_LineColor, kColorCategoria[c]));
        }
        if (v.disparo && v.estado_disparo != DISPARO_ARMADO) {
            ImPlot::TagX(v.t_disparo, kNaranja, "disparo");
        }
        dibujar_cursores(v);
        ImPlot::EndPlot();
    }
    ImPlot::EndSubplots();
    v.escalar_y = false;
}

/* Estadísticas de la ventana visible (como Signal Statistics) */
static void estadisticas(Visor &v)
{
    const size_t i0 = v.hist.buscar(v.x_min);
    const size_t i1 = v.hist.buscar(v.x_max);
    const size_t n = i1 > i0 ? i1 - i0 : 0;
    if (n == 0) {
        ImGui::TextDisabled("Sin muestras en la ventana.");
        return;
    }
    static std::vector<float> dts;
    double suma = 0.0, mn = 1e30, mx = 0.0;
    unsigned long atrasos = 0;
    dts.resize(n);
    for (size_t i = 0; i < n; i++) {
        const float d = v.hist[i0 + i].dt;
        dts[i] = d;
        suma += d;
        mn = std::min(mn, (double)d);
        mx = std::max(mx, (double)d);
        if (v.periodo_us > 0.0 && d > v.periodo_us) atrasos++;
    }
    const size_t k99 = std::min(n - 1, (size_t)std::ceil(0.99 * n) - 1);
    std::nth_element(dts.begin(), dts.begin() + k99, dts.end());
    unsigned long errores = 0;
    for (int c = 0; c < CAT_NUM; c++) {
        const Marcas &m = v.marcas[c];
        errores += std::upper_bound(m.t.begin(), m.t.end(), v.x_max) -
                   std::lower_bound(m.t.begin(), m.t.end(), v.x_min);
    }
    ImGui::TextWrapped("Ventana: %zu tramas · dt mín %.0f, media %.0f, p99 %.0f, máx %.0f us · "
                       "errores %lu · atrasos %lu", n, mn, suma / n, (double)dts[k99], mx, errores,
                       atrasos);
    if (v.cursores) {
        const double d = std::fabs(v.cursor[1] - v.cursor[0]);
        ImGui::TextColored(kCian, "Cursores: %.4f s y %.4f s, Δt %.4f s (%.2f Hz)",
                           v.cursor[0], v.cursor[1], d, d > 0.0 ? 1.0 / d : 0.0);
    }
}

/* ---- Paneles inferiores ------------------------------------------------- */

static void bytes_hex(Visor &v, const uint8_t *b, const uint8_t *dif)
{
    ImGui::PushFont(v.mono, 0.0f);
    for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) {
        if (i > 0) ImGui::SameLine(0.0f, ImGui::GetFontSize() * 0.3f);
        if (dif != NULL && dif[i] != 0) {
            ImGui::TextColored(kRojo, "%02X", b[i]);
        } else {
            ImGui::Text("%02X", b[i]);
        }
    }
    ImGui::PopFont();
}

static void detalle_bits(const TramaMala &m)
{
    ImGui::TextUnformatted("byte  recibido   esperado   distintos");
    for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) {
        if (m.diferencia[i] == 0) continue;
        char r[9], e[9], d[9];
        for (int j = 0; j < 8; j++) {
            const int bit = 7 - j;
            r[j] = (char)('0' + ((m.t.rx[i] >> bit) & 1));
            e[j] = (char)('0' + ((m.esperada[i] >> bit) & 1));
            d[j] = ((m.diferencia[i] >> bit) & 1) ? '^' : ' ';
        }
        r[8] = e[8] = d[8] = 0;
        ImGui::Text("%4u  %s   %s   %s", i, r, e, d);
    }
}

static void tabla_malas(Visor &v)
{
    const size_t n = v.bits.num_malas();
    ImGui::TextWrapped("Últimas %zu tramas con error o con respuesta distinta de la esperada. "
                       "La más reciente va arriba, los bytes distintos en rojo; el detalle por "
                       "bit aparece al pasar el ratón sobre la fila.", n);
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_ScrollX | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_SizingFixedFit;
    if (!ImGui::BeginTable("malas", 9, flags)) return;
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("trama");
    ImGui::TableSetupColumn("t [s]");
    ImGui::TableSetupColumn("dt [us]");
    ImGui::TableSetupColumn("resultado");
    ImGui::TableSetupColumn("status");
    ImGui::TableSetupColumn(v.lazo ? "MISO" : "MISO recibida");
    ImGui::TableSetupColumn(v.lazo ? "MOSI (esperada)" : "esperada");
    ImGui::TableSetupColumn("bits");
    ImGui::TableSetupColumn("status siguiente");
    ImGui::TableHeadersRow();
    ImGuiListClipper clip;
    clip.Begin((int)n);
    while (clip.Step()) {
        for (int fila = clip.DisplayStart; fila < clip.DisplayEnd; fila++) {
            const TramaMala &m = v.bits.mala(n - 1 - (size_t)fila);
            int bits = 0;
            for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) {
                for (int j = 0; j < 8; j++) bits += (m.diferencia[i] >> j) & 1;
            }
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::PushID(fila);
            ImGui::Selectable("##fila", false, ImGuiSelectableFlags_SpanAllColumns |
                                               ImGuiSelectableFlags_AllowOverlap);
            if (bits > 0 && ImGui::BeginItemTooltip()) {
                ImGui::PushFont(v.mono, 0.0f);
                detalle_bits(m);
                ImGui::PopFont();
                ImGui::EndTooltip();
            }
            ImGui::SameLine();
            ImGui::Text("%lu", m.t.k);
            ImGui::TableNextColumn();
            const double ts = v.periodo_us > 0.0 ? m.t.k * v.periodo_us * 1e-6 : m.t.t_us * 1e-6;
            ImGui::Text("%.4f", ts);
            ImGui::TableNextColumn();
            ImGui::Text("%.0f", m.t.dt_us);
            ImGui::TableNextColumn();
            ImGui::TextColored(m.t.resultado == RES_OK || m.t.resultado == RES_PRIMERA ? kBlanco : kRojo,
                               "%s", kResultados[m.t.resultado]);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(texto_status(m.t.status).c_str());
            ImGui::TableNextColumn();
            bytes_hex(v, m.t.rx, m.analizada ? m.diferencia : NULL);
            ImGui::TableNextColumn();
            if (m.analizada) bytes_hex(v, m.esperada, NULL); else ImGui::TextDisabled("-");
            ImGui::TableNextColumn();
            if (m.analizada) ImGui::Text("%d", bits); else ImGui::TextDisabled("-");
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(texto_status(m.status_siguiente).c_str());
            ImGui::PopID();
        }
    }
    ImGui::EndTable();
}

static void mapa_bits(Visor &v)
{
    static const char *const kBytesPic[DAQ_TRAMA_LEN] = {
        "0 inicio", "1 seq", "2 pos0", "3 pos1", "4 pos2", "5 pos3", "6 status", "7 CRC",
    };
    static const char *const kBytesPc[DAQ_TRAMA_LEN] = {
        "0 inicio", "1 seq", "2 DAC H", "3 DAC L", "4 flags", "5 -", "6 -", "7 CRC",
    };
    static const char *const kBits[8] = {"7", "6", "5", "4", "3", "2", "1", "0"};

    double valores[DAQ_TRAMA_LEN * 8];
    double maximo = 1.0;
    unsigned long total = 0, msb = 0;
    for (unsigned b = 0; b < DAQ_TRAMA_LEN; b++) {
        for (int j = 0; j < 8; j++) {
            const unsigned long x = v.bits.mapa[b][7 - j];
            valores[b * 8 + j] = (double)x;
            maximo = std::max(maximo, (double)x);
            total += x;
        }
        msb += v.bits.mapa[b][7];
    }
    const float alto = std::max(ImGui::GetContentRegionAvail().y, ImGui::GetFontSize() * 8.0f);
    ImPlot::PushColormap(ImPlotColormap_Hot);
    if (ImPlot::BeginPlot("##mapa", ImVec2(alto * 1.1f, alto), ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText)) {
        ImPlot::SetupAxes("bit", NULL, ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoGridLines,
                          ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoGridLines);
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, 8, ImPlotCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 8, ImPlotCond_Always);
        ImPlot::SetupAxisTicks(ImAxis_X1, 0.5, 7.5, 8, kBits);
        const char *etiquetas[DAQ_TRAMA_LEN];
        for (unsigned b = 0; b < DAQ_TRAMA_LEN; b++) {
            etiquetas[b] = (v.lazo ? kBytesPc : kBytesPic)[DAQ_TRAMA_LEN - 1 - b];
        }
        ImPlot::SetupAxisTicks(ImAxis_Y1, 0.5, 7.5, 8, etiquetas);
        ImPlot::PlotHeatmap("bits", valores, DAQ_TRAMA_LEN, 8, 0.0, maximo, "%.0f",
                            ImPlotPoint(0, 0), ImPlotPoint(8, 8));
        ImPlot::EndPlot();
    }
    ImGui::SameLine();
    ImPlot::ColormapScale("##escala", 0.0, maximo, ImVec2(0, alto));
    ImPlot::PopColormap();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::TextWrapped("Bits de la respuesta que difieren de la esperada, acumulados en la prueba "
                       "(misma lógica que herramientas/reporte).");
    ImGui::Text("Tramas analizadas: %lu", v.bits.analizadas);
    ImGui::Text("Distintas de la esperada: %lu", v.bits.distintas);
    ImGui::Text("Respuestas de arranque: %lu", v.bits.arranques);
    ImGui::Text("Bits erróneos: %lu (%lu en el bit 7)", total, msb);
    ImGui::EndGroup();
}

static void histograma(Visor &v)
{
    unsigned long total = 0;
    for (int i = 0; i < kCubetasFinas; i++) total += (unsigned long)v.fino[i];
    double p50 = 0, p99 = 0, p999 = 0;
    unsigned long acum = 0;
    for (int i = 0; i < kCubetasFinas && total > 0; i++) {
        acum += (unsigned long)v.fino[i];
        const double borde = (i + 1) * kAnchoFino;
        if (p50 == 0 && acum >= 0.5 * total) p50 = borde;
        if (p99 == 0 && acum >= 0.99 * total) p99 = borde;
        if (p999 == 0 && acum >= 0.999 * total) p999 = borde;
    }
    ImGui::Checkbox("escala log", &v.histograma_log);
    ImGui::SameLine();
    ImGui::TextWrapped("Toda la prueba: p50 < %.0f us · p99 < %.0f us · p99.9 < %.0f us · máx %.0f us",
                       p50, p99, p999, v.inst.e.c.tramas > 0 ? v.inst.e.t_max : 0.0);
    static double xs[kCubetasFinas];
    static double ys[kCubetasFinas];
    for (int i = 0; i < kCubetasFinas; i++) {
        xs[i] = (i + 0.5) * kAnchoFino;
        ys[i] = v.histograma_log && v.fino[i] == 0 ? NAN : v.fino[i];
    }
    if (ImPlot::BeginPlot("##histograma", ImVec2(-1, -1), ImPlotFlags_NoLegend)) {
        ImPlot::SetupAxes("dt [us]", "tramas", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        if (v.histograma_log) ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::PlotBars("dt", xs, ys, kCubetasFinas, kAnchoFino * 0.9,
                         ImPlotSpec(ImPlotProp_FillColor, kAmarillo, ImPlotProp_LineColor, kAmarillo));
        if (v.periodo_us > 0.0) {
            ImPlot::PlotInfLines("periodo", &v.periodo_us, 1, ImPlotSpec(ImPlotProp_LineColor, kGris));
        }
        ImPlot::EndPlot();
    }
}

static void ultima_trama(Visor &v)
{
    if (!v.hay_ultima) {
        ImGui::TextDisabled("Sin tramas.");
        return;
    }
    const Trama &t = v.ultima;
    ImGui::Text("Trama %lu", t.k);
    ImGui::Text("MOSI ");
    ImGui::SameLine();
    bytes_hex(v, t.tx, NULL);
    const unsigned dac = ((unsigned)t.tx[DAQ_PC_DAC_H] << 8) | t.tx[DAQ_PC_DAC_L];
    ImGui::TextDisabled("     seq %u, DAC %u (%.3f V), flags %02X%s%s", t.tx[DAQ_PC_SEQ], dac,
                        dac / DAQ_DAC_ESCALA - DAQ_V_MAX, t.tx[DAQ_PC_FLAGS],
                        (t.tx[DAQ_PC_FLAGS] & DAQ_FLAG_SALIDA_HAB) ? " salida" : " (0 V)",
                        (t.tx[DAQ_PC_FLAGS] & DAQ_FLAG_RESET_ENC) ? " reset" : "");
    uint8_t dif[DAQ_TRAMA_LEN];
    for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) dif[i] = t.rx[i] ^ t.tx[i];
    ImGui::Text("MISO ");
    ImGui::SameLine();
    bytes_hex(v, t.rx, v.lazo ? dif : NULL);
    ImGui::TextColored(t.resultado == RES_OK ? kGris : kRojo, "     %s", kResultados[t.resultado]);
    if (t.status >= 0) {
        ImGui::SameLine();
        ImGui::TextDisabled("eco %u, posición %ld, status %s", t.rx[DAQ_PIC_SEQ],
                            (long)t.posicion, texto_status(t.status).c_str());
    }
}

/* ---- Ventana principal -------------------------------------------------- */

static void interfaz(Visor &v)
{
    const ImGuiViewport *vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin("visor", NULL, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

    const float ancho_panel = ImGui::GetFontSize() * 28.0f;
    ImGui::BeginChild("panel", ImVec2(ancho_panel, 0), ImGuiChildFlags_Borders);
    panel_prueba(v);
    panel_estado(v);
    panel_salida(v);
    panel_vista(v);
    panel_contadores(v);
    ImGui::SeparatorText("Última trama");
    ultima_trama(v);
    ImGui::EndChild();

    ImGui::SameLine();
    ImGui::BeginChild("derecha", ImVec2(0, 0));
    const float alto = ImGui::GetContentRegionAvail().y;
    const float divisor = ImGui::GetFontSize() * 0.5f;
    const float min_displays = ImGui::GetFontSize() * 3 * 8.0f;
    const float min_pestanas = ImGui::GetFontSize() * 12.0f;
    float alto_displays = alto * v.fraccion_displays;
    alto_displays = std::max(std::min(alto_displays, alto - min_pestanas - divisor), min_displays);
    ImGui::BeginChild("displays", ImVec2(0, alto_displays));
    const float alto_estadisticas = ImGui::GetTextLineHeightWithSpacing() * (v.cursores ? 3.2f : 2.2f);
    displays(v, ImGui::GetContentRegionAvail().y - alto_estadisticas);
    estadisticas(v);
    ImGui::EndChild();

    /* Divisor arrastrable entre los displays y las pestañas */
    ImGui::InvisibleButton("##divisor", ImVec2(-1, divisor));
    if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
    }
    if (ImGui::IsItemActive() && alto > 0.0f) {
        v.fraccion_displays = std::max(0.2f, std::min(0.9f,
                                       v.fraccion_displays + ImGui::GetIO().MouseDelta.y / alto));
    }
    {
        const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
        const float y = (a.y + b.y) * 0.5f;
        ImGui::GetWindowDrawList()->AddLine(ImVec2(a.x, y), ImVec2(b.x, y),
                                            ImGui::GetColorU32(ImGuiCol_Separator), 2.0f);
    }

    if (ImGui::BeginTabBar("pestanas")) {
        if (ImGui::BeginTabItem("Tramas con error")) {
            tabla_malas(v);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Mapa de bits")) {
            mapa_bits(v);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Histograma de dt")) {
            histograma(v);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::EndChild();
    ImGui::End();
}

/* Donde el sistema ya escala el framebuffer (Wayland, macOS) no se vuelve a
 * escalar; en X11 y Windows se usa la escala del monitor */
static float escala_ventana(GLFWwindow *ventana)
{
    int vw, vh, fw, fh;
    glfwGetWindowSize(ventana, &vw, &vh);
    glfwGetFramebufferSize(ventana, &fw, &fh);
    if (fw > vw) return 1.0f;
    float x, y;
    glfwGetWindowContentScale(ventana, &x, &y);
    return x > 0.0f ? x : 1.0f;
}

static void estilo_scope(float escala)
{
    ImGui::StyleColorsDark();
    ImGuiStyle &s = ImGui::GetStyle();
    s.WindowRounding = 0.0f;
    s.FrameRounding = 3.0f;
    s.WindowPadding = ImVec2(12.0f, 12.0f);
    s.FramePadding = ImVec2(8.0f, 5.0f);
    s.ItemSpacing = ImVec2(10.0f, 8.0f);
    s.ItemInnerSpacing = ImVec2(6.0f, 6.0f);
    s.CellPadding = ImVec2(8.0f, 4.0f);
    s.SeparatorTextPadding = ImVec2(20.0f, 6.0f);
    s.Colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.10f, 0.11f, 1.0f);
    s.Colors[ImGuiCol_ChildBg] = ImVec4(0.10f, 0.10f, 0.11f, 1.0f);
    s.ScaleAllSizes(escala);
    s.FontScaleDpi = escala;

    ImPlotStyle &p = ImPlot::GetStyle();
    p.Colors[ImPlotCol_PlotBg] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
    p.Colors[ImPlotCol_FrameBg] = ImVec4(0.10f, 0.10f, 0.11f, 1.0f);
    p.Colors[ImPlotCol_PlotBorder] = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
    p.Colors[ImPlotCol_AxisGrid] = ImVec4(0.45f, 0.45f, 0.45f, 0.55f);
    p.Colors[ImPlotCol_AxisText] = ImVec4(0.85f, 0.85f, 0.85f, 1.0f);
    p.Colors[ImPlotCol_LegendBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.7f);
    p.PlotMinSize = ImVec2(100.0f, 80.0f);
    p.PlotPadding = ImVec2(10.0f, 8.0f);
    p.LabelPadding = ImVec2(6.0f, 4.0f);
}

/* Carga una fuente con acentos; si no encuentra ninguna usa la de ImGui */
static ImFont *cargar_fuente(const char *const *rutas, float tam)
{
    ImGuiIO &io = ImGui::GetIO();
    for (; *rutas != NULL; rutas++) {
        FILE *f = fopen(*rutas, "rb");
        if (f == NULL) continue;
        fclose(f);
        ImFont *fuente = io.Fonts->AddFontFromFileTTF(*rutas, tam);
        if (fuente != NULL) return fuente;
    }
    return NULL;
}

static void guardar_captura(GLFWwindow *ventana, const char *ruta)
{
    int w, h;
    glfwGetFramebufferSize(ventana, &w, &h);
    std::vector<unsigned char> px((size_t)w * h * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
    FILE *f = fopen(ruta, "wb");
    if (f == NULL) return;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = h - 1; y >= 0; y--) fwrite(&px[(size_t)y * w * 3], 1, (size_t)w * 3, f);
    fclose(f);
}

static void error_glfw(int codigo, const char *texto)
{
    fprintf(stderr, "GLFW %d: %s\n", codigo, texto);
}

int main(int argc, char **argv)
{
    Visor *v = new Visor;
    int codigo = 0;
    bool iniciar_al_abrir = false;
    bool salir = false;
    const char *captura = NULL;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-n") && i + 1 < argc) {
            v->conf.tramas = strtoul(argv[++i], NULL, 10);
        } else if (!strcmp(argv[i], "-periodo") && i + 1 < argc) {
            v->conf.periodo_us = atof(argv[++i]);
        } else if (!strcmp(argv[i], "-reloj") && i + 1 < argc) {
            v->conf.reloj = strtoul(argv[++i], NULL, 10);
        } else if (!strcmp(argv[i], "-registro") && i + 1 < argc) {
            v->conf.registrar = true;
            v->conf.ruta_registro = argv[++i];
#ifndef _WIN32
        } else if (!strcmp(argv[i], "-lazo")) {
            v->conf.modo = MODO_LAZO;
#endif
        } else if (!strcmp(argv[i], "-simulado")) {
            v->conf.modo = MODO_SIMULADO;
        } else if (!strcmp(argv[i], "-iniciar")) {
            iniciar_al_abrir = true;
        } else if (!strcmp(argv[i], "-salir")) {
            salir = true;
        } else if (!strcmp(argv[i], "-captura") && i + 1 < argc) {
            captura = argv[++i];
        } else {
            fprintf(stderr, "Uso: %s [-n tramas] [-periodo us] [-reloj Hz] [-lazo | -simulado]\n"
                            "       [-registro archivo.csv] [-iniciar] [-salir] [-captura archivo.ppm]\n",
                    argv[0]);
            return 2;
        }
    }

    glfwSetErrorCallback(error_glfw);
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    GLFWmonitor *monitor = glfwGetPrimaryMonitor();
    /* 90 % del área útil del monitor, si se conoce */
    int mx, my, mw = 0, mh = 0;
    glfwGetMonitorWorkarea(monitor, &mx, &my, &mw, &mh);
    const int ancho = mw > 0 ? mw * 9 / 10 : 1600;
    const int alto = mh > 0 ? mh * 9 / 10 : 950;
    GLFWwindow *ventana = glfwCreateWindow(ancho, alto, "Visor del enlace DAQ-SPI", NULL, NULL);
    if (ventana == NULL) return 1;
    glfwMakeContextCurrent(ventana);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = NULL;
    float escala = escala_ventana(ventana);
    estilo_scope(escala);

    static const char *const kFuentes[] = {
        "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/google-noto-vf/NotoSans[wght].ttf",
        "C:\\Windows\\Fonts\\segoeui.ttf",
        NULL,
    };
    static const char *const kFuentesMono[] = {
        "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/google-noto-vf/NotoSansMono[wght].ttf",
        "C:\\Windows\\Fonts\\consola.ttf",
        NULL,
    };
    if (cargar_fuente(kFuentes, 15.0f) == NULL) io.Fonts->AddFontDefault();
    v->mono = cargar_fuente(kFuentesMono, 15.0f);
    if (v->mono == NULL) v->mono = io.Fonts->Fonts[0];

    ImGui_ImplGlfw_InitForOpenGL(ventana, true);
    ImGui_ImplOpenGL3_Init(NULL);

    if (iniciar_al_abrir) {
        iniciar(*v);
        if (salir && !v->iniciada) {
            fprintf(stderr, "%s\n", v->mensaje.c_str());
            codigo = 2;
            glfwSetWindowShouldClose(ventana, GLFW_TRUE);
        }
    }
    const double t_abierto = glfwGetTime();

    while (!glfwWindowShouldClose(ventana)) {
        glfwPollEvents();
        if (glfwGetWindowAttrib(ventana, GLFW_ICONIFIED) != 0) {
            /* Minimizada: se siguen extrayendo tramas para no perder puntos */
            actualizar(*v);
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }
        const double t0 = glfwGetTime();
        /* La escala del framebuffer puede llegar después de crear la ventana
         * o cambiar al moverla de monitor */
        const float nueva = escala_ventana(ventana);
        if (nueva != escala) {
            escala = nueva;
            estilo_scope(escala);
        }
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        actualizar(*v);
        interfaz(*v);

        ImGui::Render();
        int w, h;
        glfwGetFramebufferSize(ventana, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.1f, 0.1f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        const double t1 = glfwGetTime();
        const bool fin_prueba = salir && v->terminada;
        if (captura != NULL && (salir ? fin_prueba : t1 - t_abierto > 3.0)) {
            guardar_captura(ventana, captura);
            if (!salir) break;
        }
        if (fin_prueba) break;
        glfwSwapBuffers(ventana);

        /* Como mucho 60 cuadros por segundo aunque el monitor sea más rápido */
        while (glfwGetTime() - v->t_cuadro < 1.0 / 60.0 - 0.0005) {
            ImGui_ImplGlfw_Sleep(1);
        }
        const double ahora = glfwGetTime();
        if (v->t_cuadro > 0.0) {
            const double ms = (ahora - v->t_cuadro) * 1e3;
            v->cuadro_ms = 0.9 * v->cuadro_ms + 0.1 * ms;
            v->cuadro_max_acum = std::max(v->cuadro_max_acum, ms);
        }
        v->trabajo_ms = 0.9 * v->trabajo_ms + 0.1 * (t1 - t0) * 1e3;
        if (ahora - v->t_ventana_max > 1.0) {
            v->cuadro_max_ms = v->cuadro_max_acum;
            v->cuadro_max_acum = 0.0;
            v->t_ventana_max = ahora;
        }
        v->t_cuadro = ahora;
    }

    v->adq.detener();
    if (salir && v->iniciada) {
        actualizar(*v);
        const Contadores &c = v->inst.e.c;
        const unsigned long errores = prueba_errores(&c);
        const double t = v->inst.transcurrido_us * 1e-6;
        printf("Tramas: %lu en %.2f s (%.0f tramas/s)\n", c.tramas, t, t > 0.0 ? c.tramas / t : 0.0);
        printf("Errores: %lu (USB %lu, eco %lu, encabezado %lu, CRC %lu, secuencia %lu; dsPIC: "
               "CRC %lu, encabezado %lu, longitud %lu, vigilancia %lu, reinicios %lu)\n",
               errores, c.err_usb, c.err_eco, c.err_inicio, c.err_crc, c.err_seq, c.pic_crc,
               c.pic_inicio, c.pic_longitud, c.pic_vigilancia, prueba_reinicios(&c));
        printf("Atrasos: %lu\n", v->inst.atrasos);
        if (c.tramas > 0) {
            printf("Duración de SPI_ReadWrite: mín %.0f us, media %.0f us, máx %.0f us\n",
                   v->inst.e.t_min, v->inst.e.t_suma / c.tramas, v->inst.e.t_max);
        }
        if (v->adq.descartadas() > 0) {
            printf("Tramas sin graficar: %lu\n", v->adq.descartadas());
        }
        printf("Resultado: %s\n", errores == 0 ? "SIN ERRORES" : "CON ERRORES");
        codigo = errores == 0 ? 0 : 1;
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(ventana);
    glfwTerminate();
    delete v;
    return codigo;
}

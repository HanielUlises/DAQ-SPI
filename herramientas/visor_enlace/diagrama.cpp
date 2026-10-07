#include "diagrama.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "imgui.h"

/* Colores del lienzo de Simulink */
static const ImU32 kFondo = IM_COL32(250, 250, 250, 255);
static const ImU32 kRejilla = IM_COL32(222, 226, 232, 255);
static const ImU32 kLinea = IM_COL32(20, 20, 20, 255);
static const ImU32 kBloque = IM_COL32(255, 255, 255, 255);
static const ImU32 kSombra = IM_COL32(0, 0, 0, 40);
static const ImU32 kTexto = IM_COL32(20, 20, 20, 255);
static const ImU32 kTextoTenue = IM_COL32(110, 110, 110, 255);
static const ImU32 kSeleccion = IM_COL32(0, 114, 217, 255);
static const ImU32 kSobre = IM_COL32(110, 170, 230, 255);
static const ImU32 kValor = IM_COL32(0, 95, 160, 255);
static const ImU32 kRojoTexto = IM_COL32(200, 30, 30, 255);
static const ImU32 kPantalla = IM_COL32(15, 15, 15, 255);
static const ImU32 kTraza = IM_COL32(240, 230, 0, 255);
static const ImU32 kDaq = IM_COL32(232, 242, 252, 255);

/* Geometría en unidades de rejilla (lazo cerrado); en lazo abierto la fuente
 * ocupa el lugar del PID y todo se recorre kRecorrido a la izquierda */
static const float kAncho = 48.5f, kAlto = 11.2f, kRecorrido = 10.5f;
static const float kY0 = 4.6f;          /* línea principal */
static const float kYfb = kY0 + 5.4f;   /* retroalimentación */

struct Caja {
    float x0, y0, x1, y1;
};

static const Caja kCajas[BLOQUE_NUM] = {
    {1.0f, kY0 - 1.5f, 6.0f, kY0 + 1.5f},           /* fuente / referencia */
    {8.1f, kY0 - 0.9f, 9.9f, kY0 + 0.9f},           /* suma */
    {11.5f, kY0 - 1.5f, 16.5f, kY0 + 1.5f},         /* PID */
    {19.0f, kY0 - 1.5f, 22.5f, kY0 + 1.5f},         /* saturación */
    {25.5f, kY0 - 1.2f, 31.5f, kY0 + 3.8f},         /* daqPic */
    {44.0f, kY0 - 4.0f, 47.5f, kY0 + 0.8f},         /* Scope */
    {33.5f, kY0 + 0.8f, 38.5f, kY0 + 2.0f},         /* errores */
    {33.5f, kY0 + 2.2f, 38.5f, kY0 + 3.4f},         /* atrasos */
};

static const float kPuertoErr = kY0 + 1.4f, kPuertoAtr = kY0 + 2.8f;
static const float kScopeR = kY0 - 3.2f, kScopeU = kY0 - 2.0f;
static const float kTapR = 7.2f, kTapU = 24.0f, kTapY = 42.0f;

const char *nombre_bloque(Bloque b, bool cerrado)
{
    switch (b) {
    case BLOQUE_FUENTE:     return cerrado ? "Referencia" : "Fuente";
    case BLOQUE_SUMA:       return "Suma";
    case BLOQUE_PID:        return "PID";
    case BLOQUE_SATURACION: return "Saturación";
    case BLOQUE_DAQ:        return "daqPic";
    case BLOQUE_SCOPE:      return "Scope";
    case BLOQUE_ERRORES:    return "Errores";
    case BLOQUE_ATRASOS:    return "Atrasos";
    default:                return "";
    }
}

/* Transformación de la rejilla a pantalla */
struct Lienzo {
    ImDrawList *dl;
    ImVec2 origen;
    float g;            /* píxeles por unidad */
    float dx;           /* recorrido del lazo abierto, en unidades */
    ImFont *fuente;

    ImVec2 p(float x, float y) const { return ImVec2(origen.x + (x + dx) * g, origen.y + y * g); }
    float grosor() const { return std::max(1.0f, g * 0.07f); }

    void texto(ImVec2 pos, const char *s, ImU32 col, float escala = 0.72f, float ax = 0.5f,
               float ay = 0.5f) const
    {
        const float tam = g * escala;
        const ImVec2 t = fuente->CalcTextSizeA(tam, 1e9f, 0.0f, s);
        dl->AddText(fuente, tam, ImVec2(pos.x - t.x * ax, pos.y - t.y * ay), col, s);
    }

    void flecha(ImVec2 punta, ImVec2 dir) const
    {
        const float l = g * 0.45f, a = g * 0.22f;
        const ImVec2 base(punta.x - dir.x * l, punta.y - dir.y * l);
        const ImVec2 n(-dir.y * a, dir.x * a);
        dl->AddTriangleFilled(punta, ImVec2(base.x + n.x, base.y + n.y),
                              ImVec2(base.x - n.x, base.y - n.y), kLinea);
    }

    void union_(float x, float y) const { dl->AddCircleFilled(p(x, y), g * 0.16f, kLinea); }

    /* Polilínea en unidades de rejilla con flecha al final */
    void cable(const ImVec2 *pts, int n) const
    {
        for (int i = 0; i + 1 < n; i++) {
            dl->AddLine(p(pts[i].x, pts[i].y), p(pts[i + 1].x, pts[i + 1].y), kLinea, grosor());
        }
        const ImVec2 a = pts[n - 2], b = pts[n - 1];
        const float lx = b.x - a.x, ly = b.y - a.y, l = std::sqrt(lx * lx + ly * ly);
        if (l > 0.0f) flecha(p(b.x, b.y), ImVec2(lx / l, ly / l));
    }

    void recta(float x0, float x1, float y) const
    {
        const ImVec2 pts[2] = {ImVec2(x0, y), ImVec2(x1, y)};
        cable(pts, 2);
    }

    /* Etiqueta de una señal sobre la línea, con su valor si lo hay */
    void senal(float x, float y, const char *nombre, const char *valor) const
    {
        texto(p(x, y - 0.25f), nombre, kTexto, 0.72f, 0.5f, 1.0f);
        if (valor != NULL) texto(p(x, y + 0.25f), valor, kValor, 0.62f, 0.5f, 0.0f);
    }
};

static void glifo_fuente(const Lienzo &l, const Caja &c, Forma forma)
{
    const float mx = 0.8f, my = 0.7f;
    const float x0 = c.x0 + mx, x1 = c.x1 - mx, ym = (c.y0 + c.y1) * 0.5f;
    const float a = (c.y1 - c.y0) * 0.5f - my;
    const float w = x1 - x0;
    ImVec2 pts[64];
    int n = 0;
    switch (forma) {
    case FORMA_SENO:
        for (int i = 0; i < 48; i++) {
            const float f = i / 47.0f;
            pts[n++] = l.p(x0 + f * w, ym - a * std::sin(f * 2.0f * (float)M_PI));
        }
        break;
    case FORMA_CUADRADA:
        pts[n++] = l.p(x0, ym + a);
        pts[n++] = l.p(x0, ym - a);
        pts[n++] = l.p(x0 + w * 0.5f, ym - a);
        pts[n++] = l.p(x0 + w * 0.5f, ym + a);
        pts[n++] = l.p(x1, ym + a);
        pts[n++] = l.p(x1, ym - a);
        break;
    case FORMA_RAMPA:
        pts[n++] = l.p(x0, ym + a);
        pts[n++] = l.p(x0 + w * 0.5f, ym - a);
        pts[n++] = l.p(x1, ym + a);
        break;
    case FORMA_ESCALON:
        pts[n++] = l.p(x0, ym + a);
        pts[n++] = l.p(x0 + w * 0.4f, ym + a);
        pts[n++] = l.p(x0 + w * 0.4f, ym - a);
        pts[n++] = l.p(x1, ym - a);
        break;
    default:
        pts[n++] = l.p(x0, ym);
        pts[n++] = l.p(x1, ym);
        break;
    }
    l.dl->AddPolyline(pts, n, kLinea, 0, l.grosor());
}

static void glifo_saturacion(const Lienzo &l, const Caja &c)
{
    const float xm = (c.x0 + c.x1) * 0.5f, ym = (c.y0 + c.y1) * 0.5f;
    const float a = 1.0f;
    l.dl->AddLine(l.p(c.x0 + 0.4f, ym), l.p(c.x1 - 0.4f, ym), kTextoTenue, 1.0f);
    l.dl->AddLine(l.p(xm, c.y0 + 0.3f), l.p(xm, c.y1 - 0.3f), kTextoTenue, 1.0f);
    const ImVec2 pts[4] = {l.p(c.x0 + 0.4f, ym + a * 0.6f), l.p(xm - 0.6f, ym + a * 0.6f),
                           l.p(xm + 0.6f, ym - a * 0.6f), l.p(c.x1 - 0.4f, ym - a * 0.6f)};
    l.dl->AddPolyline(pts, 4, kLinea, 0, l.grosor());
}

static void glifo_scope(const Lienzo &l, const Caja &c)
{
    const ImVec2 a = l.p(c.x0 + 0.5f, c.y0 + 0.6f), b = l.p(c.x1 - 0.5f, c.y1 - 1.4f);
    l.dl->AddRectFilled(a, b, kPantalla, l.g * 0.15f);
    ImVec2 pts[32];
    for (int i = 0; i < 32; i++) {
        const float f = i / 31.0f;
        pts[i] = ImVec2(a.x + (b.x - a.x) * (0.1f + 0.8f * f),
                        (a.y + b.y) * 0.5f - (b.y - a.y) * 0.3f * std::sin(f * 9.0f) * (1.0f - 0.6f * f));
    }
    l.dl->AddPolyline(pts, 32, kTraza, 0, std::max(1.0f, l.g * 0.08f));
    /* Perilla y botones del frente */
    l.dl->AddCircle(l.p(c.x0 + 1.0f, c.y1 - 0.75f), l.g * 0.28f, kLinea, 0, 1.0f);
    l.dl->AddRect(l.p(c.x1 - 1.8f, c.y1 - 1.0f), l.p(c.x1 - 0.5f, c.y1 - 0.5f), kLinea, 0.0f, 0, 1.0f);
}

/* Marca de puerto de entrada (triángulo hacia dentro) o de salida */
static void puerto(const Lienzo &l, float x, float y, bool entrada)
{
    const float s = 0.28f;
    if (entrada) {
        l.dl->AddTriangleFilled(l.p(x, y - s), l.p(x + s * 1.1f, y), l.p(x, y + s), kLinea);
    } else {
        l.dl->AddTriangleFilled(l.p(x - s * 1.1f, y - s), l.p(x, y), l.p(x - s * 1.1f, y + s), kLinea);
    }
}

float alto_diagrama(float ancho)
{
    const float g = std::min(ancho / kAncho, ImGui::GetFontSize() * 1.15f);
    return kAlto * g;
}

Bloque diagrama(const DatosDiagrama &d, Bloque seleccion)
{
    const float ancho = ImGui::GetContentRegionAvail().x;
    const float g = std::min(ancho / kAncho, ImGui::GetFontSize() * 1.15f);
    const ImVec2 esquina = ImGui::GetCursorScreenPos();
    const ImVec2 tam(ancho, kAlto * g);

    Lienzo l;
    l.dl = ImGui::GetWindowDrawList();
    l.g = g;
    l.dx = d.cerrado ? 0.0f : -kRecorrido;
    l.fuente = ImGui::GetFont();
    const float ancho_modelo = d.cerrado ? kAncho : kAncho - kRecorrido;
    l.origen = ImVec2(esquina.x + std::max(0.0f, (ancho - ancho_modelo * g) * 0.5f), esquina.y);

    /* Fondo con rejilla de puntos */
    l.dl->AddRectFilled(esquina, ImVec2(esquina.x + tam.x, esquina.y + tam.y), kFondo);
    l.dl->PushClipRect(esquina, ImVec2(esquina.x + tam.x, esquina.y + tam.y), true);
    for (float x = esquina.x + g; x < esquina.x + tam.x; x += g) {
        for (float y = esquina.y + g; y < esquina.y + tam.y; y += g) {
            l.dl->AddRectFilled(ImVec2(x, y), ImVec2(x + 1.0f, y + 1.0f), kRejilla);
        }
    }

    /* Valores de las señales sobre las líneas */
    char vr[32], ve[32], vu[32], vy[32];
    const char *pr = NULL, *pe = NULL, *pu = NULL, *py = NULL;
    if (d.hay_valores) {
        snprintf(vr, sizeof vr, "%.4g %s", d.r, d.unidad);
        snprintf(ve, sizeof ve, "%.4g %s", d.e, d.unidad);
        snprintf(vu, sizeof vu, "%.3f V", d.u);
        snprintf(vy, sizeof vy, "%.4g %s", d.y, d.unidad);
        pu = vu;
        py = vy;
        if (std::isfinite(d.r)) {
            pr = vr;
            pe = ve;
        }
    }

    /* ---- Líneas ---- */
    const Caja &cf = kCajas[BLOQUE_FUENTE], &cs = kCajas[BLOQUE_SUMA], &cp = kCajas[BLOQUE_PID];
    const Caja &ct = kCajas[BLOQUE_SATURACION], &cd = kCajas[BLOQUE_DAQ], &cc = kCajas[BLOQUE_SCOPE];
    const Caja &ce = kCajas[BLOQUE_ERRORES], &ca = kCajas[BLOQUE_ATRASOS];
    if (d.cerrado) {
        l.recta(cf.x1, cs.x0, kY0);
        l.union_(kTapR, kY0);
        const ImVec2 r_scope[3] = {ImVec2(kTapR, kY0), ImVec2(kTapR, kScopeR), ImVec2(cc.x0, kScopeR)};
        l.cable(r_scope, 3);
        l.senal(13.0f, kScopeR, "r", pr);
        l.recta(cs.x1, cp.x0, kY0);
        l.senal((cs.x1 + cp.x0) * 0.5f, kY0, "e", NULL);
        l.recta(cp.x1, ct.x0, kY0);
        /* Retroalimentación */
        l.union_(kTapY, kY0);
        const ImVec2 fb[4] = {ImVec2(kTapY, kY0), ImVec2(kTapY, kYfb), ImVec2(9.0f, kYfb),
                              ImVec2(9.0f, cs.y1)};
        l.cable(fb, 4);
        if (pe != NULL) l.texto(l.p((cs.x1 + cp.x0) * 0.5f, kY0 + 0.25f), pe, kValor, 0.62f, 0.5f, 0.0f);
    } else {
        l.recta(cp.x1, ct.x0, kY0);
    }
    l.recta(ct.x1, cd.x0, kY0);
    l.union_(kTapU, kY0);
    const ImVec2 u_scope[3] = {ImVec2(kTapU, kY0), ImVec2(kTapU, kScopeU), ImVec2(cc.x0, kScopeU)};
    l.cable(u_scope, 3);
    l.senal(28.5f, kScopeU, "u", pu);
    l.recta(cd.x1, cc.x0, kY0);
    l.senal(36.0f, kY0, "y", py);
    l.recta(cd.x1, ce.x0, kPuertoErr);
    l.recta(cd.x1, ca.x0, kPuertoAtr);

    /* ---- Bloques ---- */
    Bloque clic = BLOQUE_NINGUNO;
    for (int i = 0; i < BLOQUE_NUM; i++) {
        const Bloque b = (Bloque)i;
        if (!d.cerrado && (b == BLOQUE_SUMA || b == BLOQUE_FUENTE)) continue;
        /* En lazo abierto la fuente ocupa la caja del PID */
        const Bloque dibujo = !d.cerrado && b == BLOQUE_PID ? BLOQUE_FUENTE : b;
        const Caja &c = kCajas[b];
        const ImVec2 a = l.p(c.x0, c.y0), z = l.p(c.x1, c.y1);

        ImGui::SetCursorScreenPos(a);
        ImGui::PushID(i);
        ImGui::InvisibleButton("##bloque", ImVec2(z.x - a.x, z.y - a.y));
        const bool sobre = ImGui::IsItemHovered();
        if (ImGui::IsItemClicked()) clic = dibujo;
        if (sobre) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            ImGui::SetTooltip("%s: clic para ver sus parámetros", nombre_bloque(dibujo, d.cerrado));
        }
        ImGui::PopID();

        const bool sel = seleccion == dibujo;
        const ImU32 borde = sel ? kSeleccion : sobre ? kSobre : kLinea;
        const float grosor = sel || sobre ? l.grosor() * 2.2f : l.grosor();
        const float sombra = g * 0.15f;
        if (b == BLOQUE_SUMA) {
            const ImVec2 cen = l.p(9.0f, kY0);
            l.dl->AddCircleFilled(ImVec2(cen.x + sombra, cen.y + sombra), g * 0.9f, kSombra);
            l.dl->AddCircleFilled(cen, g * 0.9f, kBloque);
            l.dl->AddCircle(cen, g * 0.9f, borde, 0, grosor);
            l.texto(l.p(8.5f, kY0 - 0.15f), "+", kTexto, 0.8f);
            l.texto(l.p(9.0f, kY0 + 0.45f), "−", kTexto, 0.8f);
            continue;
        }
        const ImU32 relleno = b == BLOQUE_DAQ ? kDaq : kBloque;
        l.dl->AddRectFilled(ImVec2(a.x + sombra, a.y + sombra), ImVec2(z.x + sombra, z.y + sombra), kSombra);
        l.dl->AddRectFilled(a, z, relleno);
        l.dl->AddRect(a, z, borde, 0.0f, 0, grosor);

        const float xm = (c.x0 + c.x1) * 0.5f, ym = (c.y0 + c.y1) * 0.5f;
        switch (dibujo) {
        case BLOQUE_FUENTE:
            glifo_fuente(l, c, d.forma);
            puerto(l, c.x1, kY0, false);
            break;
        case BLOQUE_PID:
            l.texto(l.p(xm, ym), "PID(z)", kTexto, 0.85f);
            puerto(l, c.x0, kY0, true);
            puerto(l, c.x1, kY0, false);
            break;
        case BLOQUE_SATURACION:
            glifo_saturacion(l, c);
            puerto(l, c.x0, kY0, true);
            puerto(l, c.x1, kY0, false);
            break;
        case BLOQUE_DAQ:
            l.texto(l.p(xm, c.y0 + 1.2f), "daqPic", kTexto, 0.9f);
            l.texto(l.p(xm, c.y0 + 2.1f), d.modo, kTextoTenue, 0.6f);
            l.texto(l.p(c.x0 + 0.5f, kY0), "V", kTexto, 0.6f, 0.0f);
            l.texto(l.p(c.x1 - 0.5f, kY0), "pos", kTexto, 0.6f, 1.0f);
            l.texto(l.p(c.x1 - 0.5f, kPuertoErr), "err", kTexto, 0.6f, 1.0f);
            l.texto(l.p(c.x1 - 0.5f, kPuertoAtr), "atr", kTexto, 0.6f, 1.0f);
            puerto(l, c.x0, kY0, true);
            puerto(l, c.x1, kY0, false);
            puerto(l, c.x1, kPuertoErr, false);
            puerto(l, c.x1, kPuertoAtr, false);
            break;
        case BLOQUE_SCOPE:
            glifo_scope(l, c);
            if (d.cerrado) puerto(l, c.x0, kScopeR, true);
            puerto(l, c.x0, kScopeU, true);
            puerto(l, c.x0, kY0, true);
            break;
        case BLOQUE_ERRORES:
        case BLOQUE_ATRASOS: {
            char buf[24];
            const unsigned long n = dibujo == BLOQUE_ERRORES ? d.errores : d.atrasos;
            snprintf(buf, sizeof buf, "%lu", n);
            l.texto(l.p(c.x1 - 0.4f, ym), buf, n > 0 ? kRojoTexto : kTexto, 0.75f, 1.0f);
            puerto(l, c.x0, ym, true);
            break;
        }
        default:
            break;
        }
        /* Nombre debajo del bloque, como en Simulink */
        if (dibujo == BLOQUE_ERRORES || dibujo == BLOQUE_ATRASOS) {
            l.texto(l.p(c.x1 + 0.3f, ym), nombre_bloque(dibujo, d.cerrado), kTextoTenue, 0.6f, 0.0f);
        } else {
            l.texto(l.p(xm, c.y1 + 0.2f), nombre_bloque(dibujo, d.cerrado), kTexto, 0.65f, 0.5f, 0.0f);
        }
    }
    l.dl->PopClipRect();

    ImGui::SetCursorScreenPos(esquina);
    ImGui::Dummy(tam);
    return clic;
}

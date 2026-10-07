/*
 * Exportación de las señales del visor, como el bloque To Workspace:
 *   - .mat: formato MAT 4 (load en MATLAB u Octave, scipy.io.loadmat), una
 *     variable columna de tipo double por señal;
 *   - cualquier otra extensión: CSV con una columna por señal y el
 *     comentario como encabezado (líneas que empiezan con #).
 */
#ifndef EXPORTAR_H
#define EXPORTAR_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

struct Columna {
    std::string nombre;
    std::vector<double> datos;
};

static inline bool termina_en(const std::string &s, const char *sufijo)
{
    const size_t n = strlen(sufijo);
    if (s.size() < n) return false;
    for (size_t i = 0; i < n; i++) {
        char c = s[s.size() - n + i];
        if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        if (c != sufijo[i]) return false;
    }
    return true;
}

/* Una variable de MAT 4: tipo 0 (double, little-endian, numérica completa),
 * filas, columnas, sin parte imaginaria, nombre con su terminador y los datos
 * por columnas. Se asume una PC little-endian. */
static inline bool mat4_variable(FILE *f, const Columna &c)
{
    const int32_t encabezado[5] = {0, (int32_t)c.datos.size(), 1, 0, (int32_t)c.nombre.size() + 1};
    if (fwrite(encabezado, sizeof encabezado, 1, f) != 1) return false;
    if (fwrite(c.nombre.c_str(), c.nombre.size() + 1, 1, f) != 1) return false;
    if (c.datos.empty()) return true;
    return fwrite(c.datos.data(), sizeof(double), c.datos.size(), f) == c.datos.size();
}

/* Escribe las columnas en ruta; si falla deja el motivo en error */
static inline bool exportar_senales(const std::string &ruta, const std::vector<Columna> &columnas,
                                    const std::string &comentario, std::string &error)
{
    const bool mat = termina_en(ruta, ".mat");
    FILE *f = fopen(ruta.c_str(), mat ? "wb" : "w");
    if (f == NULL) {
        error = "No se pudo crear " + ruta + ".";
        return false;
    }
    bool ok = true;
    if (mat) {
        for (const Columna &c : columnas) ok = ok && mat4_variable(f, c);
    } else {
        size_t pos = 0;
        while (pos < comentario.size()) {
            size_t fin = comentario.find('\n', pos);
            if (fin == std::string::npos) fin = comentario.size();
            fprintf(f, "# %s\n", comentario.substr(pos, fin - pos).c_str());
            pos = fin + 1;
        }
        size_t filas = 0;
        for (size_t i = 0; i < columnas.size(); i++) {
            fprintf(f, "%s%s", i > 0 ? "," : "", columnas[i].nombre.c_str());
            if (columnas[i].datos.size() > filas) filas = columnas[i].datos.size();
        }
        fputc('\n', f);
        for (size_t k = 0; k < filas; k++) {
            for (size_t i = 0; i < columnas.size(); i++) {
                if (i > 0) fputc(',', f);
                if (k < columnas[i].datos.size()) fprintf(f, "%.9g", columnas[i].datos[k]);
            }
            fputc('\n', f);
        }
        ok = !ferror(f);
    }
    if (fclose(f) != 0) ok = false;
    if (!ok) error = "Falló la escritura de " + ruta + ".";
    return ok;
}

#endif /* EXPORTAR_H */

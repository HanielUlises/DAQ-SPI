/*
 * Exportación de las señales del visor, como el bloque To Workspace:
 *   - .mat: formato MAT 4 (load en MATLAB u Octave, scipy.io.loadmat), una
 *     variable columna de tipo double por señal;
 *   - cualquier otra extensión: CSV con una columna por señal y el
 *     comentario como encabezado (líneas que empiezan con #).
 *
 * importar_senales() lee lo mismo de regreso: un .mat en MAT 4 (el de
 * exportar_senales o el que MATLAB guarda con save -v4, como
 * SIMULINK/DaqPic/crear_modelo_pid.m) o un CSV; en el CSV, una línea de
 * comentario "escala = X" se lee como la variable escala.
 */
#ifndef EXPORTAR_H
#define EXPORTAR_H

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

/* Lee una variable de MAT 4; devuelve 1 si la leyó, 0 al final del archivo
 * y -1 si hay un error (el motivo queda en error) */
static inline int mat4_leer(FILE *f, Columna &c, bool &usable, std::string &error)
{
    int32_t h[5];
    const size_t leidos = fread(h, sizeof(int32_t), 5, f);
    if (leidos == 0) return 0;
    if (leidos != 5) {
        error = "Archivo MAT truncado.";
        return -1;
    }
    const int32_t tipo = h[0], filas = h[1], columnas = h[2], imag = h[3], largo = h[4];
    const int m = tipo / 1000, o = (tipo / 100) % 10, p = (tipo / 10) % 10, t = tipo % 10;
    if (tipo < 0 || tipo > 9999 || m != 0 || o != 0 || filas < 0 || columnas < 0 || largo <= 0 ||
        largo > 4096) {
        error = "No es un archivo MAT 4 little-endian (en MATLAB: save(archivo, ..., '-v4')).";
        return -1;
    }
    static const size_t kTam[6] = {8, 4, 4, 2, 2, 1};
    if (p > 5) {
        error = "Tipo de dato desconocido en el archivo MAT.";
        return -1;
    }
    std::vector<char> nombre((size_t)largo);
    if (fread(nombre.data(), 1, nombre.size(), f) != nombre.size()) {
        error = "Archivo MAT truncado.";
        return -1;
    }
    nombre.back() = 0;
    c.nombre = nombre.data();
    const size_t n = (size_t)filas * (size_t)columnas;
    std::vector<unsigned char> crudo(n * kTam[p]);
    if (!crudo.empty() && fread(crudo.data(), 1, crudo.size(), f) != crudo.size()) {
        error = "Archivo MAT truncado.";
        return -1;
    }
    if (imag != 0 && fseek(f, (long)crudo.size(), SEEK_CUR) != 0) {
        error = "Archivo MAT truncado.";
        return -1;
    }
    /* Texto (t = 1) y matrices dispersas (t = 2) no son señales */
    usable = t == 0;
    c.datos.resize(n);
    for (size_t i = 0; i < n && usable; i++) {
        const unsigned char *b = &crudo[i * kTam[p]];
        double v = 0.0;
        switch (p) {
        case 0: { double x; memcpy(&x, b, 8); v = x; break; }
        case 1: { float x; memcpy(&x, b, 4); v = x; break; }
        case 2: { int32_t x; memcpy(&x, b, 4); v = x; break; }
        case 3: { int16_t x; memcpy(&x, b, 2); v = x; break; }
        case 4: { uint16_t x; memcpy(&x, b, 2); v = x; break; }
        default: v = b[0]; break;
        }
        c.datos[i] = v;
    }
    return 1;
}

/* Lee las señales de ruta (.mat en MAT 4 o CSV); si falla deja el motivo en error */
static inline bool importar_senales(const std::string &ruta, std::vector<Columna> &columnas,
                                    std::string &error)
{
    columnas.clear();
    const bool mat = termina_en(ruta, ".mat");
    FILE *f = fopen(ruta.c_str(), mat ? "rb" : "r");
    if (f == NULL) {
        error = "No se pudo abrir " + ruta + ".";
        return false;
    }
    bool ok = true;
    if (mat) {
        char firma[20] = {0};
        if (fread(firma, 1, sizeof firma - 1, f) > 0 && strncmp(firma, "MATLAB ", 7) == 0) {
            error = "El archivo es MAT 5, 7 o 7.3: en MATLAB guárdalo con save(archivo, ..., '-v4').";
            fclose(f);
            return false;
        }
        rewind(f);
        for (;;) {
            Columna c;
            bool usable = false;
            const int r = mat4_leer(f, c, usable, error);
            if (r < 0) ok = false;
            if (r <= 0) break;
            if (usable) columnas.push_back(c);
        }
    } else {
        std::string linea;
        bool encabezado = false;
        size_t inicio = 0, ancho = 0;   /* columnas del encabezado en columnas[inicio..] */
        int ch;
        for (;;) {
            linea.clear();
            while ((ch = fgetc(f)) != EOF && ch != '\n') {
                if (ch != '\r') linea += (char)ch;
            }
            if (linea.empty() && ch == EOF) break;
            if (linea.empty()) continue;
            if (linea[0] == '#') {
                double x;
                if (sscanf(linea.c_str(), "# escala = %lf", &x) == 1) {
                    Columna c;
                    c.nombre = "escala";
                    c.datos.push_back(x);
                    columnas.push_back(c);
                }
                continue;
            }
            /* Campos separados por comas */
            std::vector<std::string> campos;
            size_t pos = 0;
            for (;;) {
                const size_t fin = linea.find(',', pos);
                campos.push_back(linea.substr(pos, fin == std::string::npos ? std::string::npos : fin - pos));
                if (fin == std::string::npos) break;
                pos = fin + 1;
            }
            if (!encabezado) {
                encabezado = true;
                inicio = columnas.size();
                ancho = campos.size();
                for (const std::string &nombre : campos) {
                    Columna c;
                    c.nombre = nombre;
                    columnas.push_back(c);
                }
                continue;
            }
            for (size_t i = 0; i < ancho; i++) {
                double x = NAN;
                if (i < campos.size()) {
                    char *fin = NULL;
                    x = strtod(campos[i].c_str(), &fin);
                    if (fin == campos[i].c_str()) x = NAN;
                }
                columnas[inicio + i].datos.push_back(x);
            }
        }
        if (!encabezado) {
            error = "El CSV no tiene encabezado.";
            ok = false;
        }
    }
    fclose(f);
    return ok;
}

/* Columna con ese nombre, o NULL */
static inline const Columna *buscar_columna(const std::vector<Columna> &columnas, const char *nombre)
{
    for (const Columna &c : columnas) {
        if (c.nombre == nombre) return &c;
    }
    return NULL;
}

#endif /* EXPORTAR_H */

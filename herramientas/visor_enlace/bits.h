/*
 * Análisis de bits erróneos para el visor: compara cada respuesta del dsPIC
 * con la que debió llegar y acumula un mapa byte x bit de diferencias, más
 * un historial de las últimas tramas con algún problema.
 *
 * Sigue la misma lógica que herramientas/reporte/reporte.py:
 *  - En lazo, la respuesta esperada es la trama enviada.
 *  - Con el dsPIC, no se analizan la trama k = 0, las respuestas sin el
 *    encabezado 0x5A ni las de arranque (secuencia 0 y posición 0 cuando la
 *    trama anterior no tenía secuencia 0), que se cuentan aparte.
 *  - La esperada es 5A, el eco de la secuencia de la trama k-1, k-2 o k-3
 *    (si el dsPIC rechazó las anteriores), la posición y un status de entre
 *    {0, 1, 2, 4, 8, 9, 10, 12, el recibido}, con su CRC; se toma la de
 *    menor distancia de Hamming.
 * La posición candidata es la de la última respuesta válida y, para que un
 * motor en movimiento no aparezca como bits erróneos, también su
 * extrapolación con la velocidad de las dos últimas respuestas válidas, con
 * +/-1 y +/-2 cuentas que absorben la aceleración y el redondeo. Esto sólo
 * mientras el motor se mueve (la posición cambió en las últimas
 * kVentanaMovimiento tramas); con el motor quieto queda únicamente la última
 * posición, como en reporte.py. En movimiento, a cambio, una inversión de los
 * bits 0 o 1 de la posición puede pasar inadvertida.
 *
 * Una trama llega con orden creciente de k. Si se salta alguna (el anillo de
 * la adquisición descartó tramas), se reinicia el contexto y esa trama no se
 * analiza. El costo por trama es constante.
 */
#ifndef BITS_H
#define BITS_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "../comun/enlace_prueba.h"

struct TramaMala {
    Trama t;                                /* tal como se recibió */
    uint8_t esperada[DAQ_TRAMA_LEN];
    uint8_t diferencia[DAQ_TRAMA_LEN];      /* esperada ^ t.rx; 0 si no se analizó */
    bool analizada;                         /* false: arranque, encabezado malo, k = 0... */
    int status_siguiente;                   /* status de la trama k+1; -1 si no llega o no es válida */
};

class AnalisisBits {
public:
    explicit AnalisisBits(size_t max_malas = 200)
        : malas_(max_malas > 0 ? max_malas : 1)
    {
        reiniciar(false);
    }

    /* Al iniciar cada prueba */
    void reiniciar(bool lazo)
    {
        lazo_ = lazo;
        memset(mapa, 0, sizeof mapa);
        analizadas = distintas = arranques = 0;
        inicio_ = num_ = 0;
        olvidar_contexto();
    }

    void agregar(const Trama &t)
    {
        if (hay_ant1_ && t.k != k_ant1_ + 1u) {
            olvidar_contexto();
        }

        /* La bandera de longitud de una transferencia llega en la respuesta
         * siguiente: se anota en la trama mala anterior. */
        if (pendiente_ && t.k == k_pendiente_ + 1u && malas_[i_pendiente_].t.k == k_pendiente_) {
            malas_[i_pendiente_].status_siguiente = t.status;
        }
        pendiente_ = false;

        uint8_t esperada[DAQ_TRAMA_LEN];
        bool analizada = false;
        if (lazo_) {
            memcpy(esperada, t.tx, DAQ_TRAMA_LEN);
            analizada = true;
        } else if (hay_ant1_ && es_arranque(t)) {
            arranques++;
        } else if (t.k != 0u && hay_ant1_ && t.rx[DAQ_PIC_INICIO] == DAQ_INICIO_PIC) {
            if (t.resultado == RES_OK || t.resultado == RES_SEQ) {
                anotar_posicion(t.k, t.posicion);
            }
            esperar(t, esperada);
            analizada = true;
        }

        uint8_t diferencia[DAQ_TRAMA_LEN] = {0};
        bool distinta = false;
        if (analizada) {
            analizadas++;
            for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) {
                diferencia[i] = (uint8_t)(esperada[i] ^ t.rx[i]);
                for (unsigned b = 0; b < 8u; b++) {
                    if ((diferencia[i] >> b) & 1u) mapa[i][b]++;
                }
                if (diferencia[i] != 0u) distinta = true;
            }
            if (distinta) distintas++;
        }

        const bool mala = (t.resultado != RES_OK && t.resultado != RES_PRIMERA) ||
                          (t.status > 0) || distinta;
        if (mala) {
            const size_t i = guardar();
            TramaMala &m = malas_[i];
            m.t = t;
            if (analizada) {
                memcpy(m.esperada, esperada, DAQ_TRAMA_LEN);
            } else {
                memset(m.esperada, 0, DAQ_TRAMA_LEN);
            }
            memcpy(m.diferencia, diferencia, DAQ_TRAMA_LEN);
            m.analizada = analizada;
            m.status_siguiente = -1;
            pendiente_ = true;
            i_pendiente_ = i;
            k_pendiente_ = t.k;
        }

        /* Contexto para la trama siguiente */
        hay_ant3_ = hay_ant2_;
        seq_ant3_ = seq_ant2_;
        hay_ant2_ = hay_ant1_;
        seq_ant2_ = seq_ant1_;
        hay_ant1_ = true;
        seq_ant1_ = t.tx[DAQ_PC_SEQ];
        k_ant1_ = t.k;
    }

    unsigned long mapa[DAQ_TRAMA_LEN][8];   /* [byte][bit], bit 0 = LSB */
    unsigned long analizadas;
    unsigned long distintas;
    unsigned long arranques;

    size_t num_malas() const { return num_; }

    /* 0 = la más antigua, num_malas() - 1 = la más reciente */
    const TramaMala &mala(size_t i) const { return malas_[(inicio_ + i) % malas_.size()]; }

private:
    std::vector<TramaMala> malas_;
    size_t inicio_ = 0;
    size_t num_ = 0;
    bool lazo_ = false;

    bool hay_ant1_ = false, hay_ant2_ = false, hay_ant3_ = false;
    uint8_t seq_ant1_ = 0, seq_ant2_ = 0, seq_ant3_ = 0;
    unsigned long k_ant1_ = 0;

    /* Posición: última respuesta válida y la anterior, para extrapolar */
    static const unsigned long kVentanaMovimiento = 200;
    int num_pos_ = 0;
    int32_t pos1_ = 0, pos2_ = 0;
    unsigned long k_pos1_ = 0, k_pos2_ = 0;
    bool movio_ = false;
    unsigned long k_movimiento_ = 0;    /* última trama en que cambió la posición */

    bool pendiente_ = false;
    size_t i_pendiente_ = 0;
    unsigned long k_pendiente_ = 0;

    void olvidar_contexto()
    {
        hay_ant1_ = hay_ant2_ = hay_ant3_ = false;
        num_pos_ = 0;
        pos1_ = pos2_ = 0;
        movio_ = false;
        pendiente_ = false;
    }

    bool es_arranque(const Trama &t) const
    {
        static const uint8_t kArranque[6] = {DAQ_INICIO_PIC, 0, 0, 0, 0, 0};
        return memcmp(t.rx, kArranque, sizeof kArranque) == 0 && seq_ant1_ != 0u;
    }

    void anotar_posicion(unsigned long k, int32_t pos)
    {
        if (num_pos_ > 0 && pos != pos1_) {
            movio_ = true;
            k_movimiento_ = k;
        }
        pos2_ = pos1_;
        k_pos2_ = k_pos1_;
        pos1_ = pos;
        k_pos1_ = k;
        if (num_pos_ < 2) num_pos_++;
    }

    static void respuesta(uint8_t *r, uint8_t seq, int32_t pos, uint8_t status)
    {
        const uint32_t p = (uint32_t)pos;
        r[DAQ_PIC_INICIO]   = DAQ_INICIO_PIC;
        r[DAQ_PIC_SEQ]      = seq;
        r[DAQ_PIC_ENC0]     = (uint8_t)p;
        r[DAQ_PIC_ENC0 + 1] = (uint8_t)(p >> 8);
        r[DAQ_PIC_ENC0 + 2] = (uint8_t)(p >> 16);
        r[DAQ_PIC_ENC0 + 3] = (uint8_t)(p >> 24);
        r[DAQ_PIC_STATUS]   = status;
        r[DAQ_PIC_CRC]      = daq_crc8(r, DAQ_TRAMA_LEN - 1u);
    }

    static unsigned distancia(const uint8_t *a, const uint8_t *b)
    {
        unsigned d = 0;
        for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) {
            uint8_t x = (uint8_t)(a[i] ^ b[i]);
            while (x) {
                d += x & 1u;
                x >>= 1;
            }
        }
        return d;
    }

    /* Respuesta candidata más cercana a la recibida */
    void esperar(const Trama &t, uint8_t *esperada) const
    {
        uint8_t seqs[3];
        int num_seqs = 0;
        seqs[num_seqs++] = seq_ant1_;
        if (hay_ant2_) seqs[num_seqs++] = seq_ant2_;
        if (hay_ant3_) seqs[num_seqs++] = seq_ant3_;

        int32_t posiciones[6];
        int num_posiciones = 0;
        posiciones[num_posiciones++] = pos1_;
        if (movio_ && t.k - k_movimiento_ <= kVentanaMovimiento) {
            double p = (double)pos1_;
            if (num_pos_ == 2 && k_pos1_ > k_pos2_ && t.k > k_pos1_) {
                const double v = (double)(pos1_ - pos2_) / (double)(k_pos1_ - k_pos2_);
                p += v * (double)(t.k - k_pos1_);
            }
            if (p > -2147483600.0 && p < 2147483600.0) {
                const int32_t extrapolada = (int32_t)(p < 0.0 ? p - 0.5 : p + 0.5);
                static const int32_t kVecinas[5] = {0, -1, 1, -2, 2};
                for (int i = 0; i < 5; i++) {
                    const int32_t c = extrapolada + kVecinas[i];
                    if (c != pos1_) posiciones[num_posiciones++] = c;
                }
            }
        }

        const uint8_t estados[9] = {0u, 1u, 2u, 4u, 8u, 9u, 10u, 12u, t.rx[DAQ_PIC_STATUS]};
        unsigned mejor = 1000u;
        uint8_t candidata[DAQ_TRAMA_LEN];
        for (int s = 0; s < num_seqs; s++) {
            for (int p = 0; p < num_posiciones; p++) {
                for (unsigned e = 0; e < 9u; e++) {
                    respuesta(candidata, seqs[s], posiciones[p], estados[e]);
                    const unsigned d = distancia(candidata, t.rx);
                    if (d < mejor) {
                        mejor = d;
                        memcpy(esperada, candidata, DAQ_TRAMA_LEN);
                        if (d == 0u) return;
                    }
                }
            }
        }
    }

    /* Siguiente casilla del historial circular */
    size_t guardar()
    {
        const size_t capacidad = malas_.size();
        if (num_ < capacidad) {
            return (inicio_ + num_++) % capacidad;
        }
        const size_t i = inicio_;
        inicio_ = (inicio_ + 1u) % capacidad;
        return i;
    }
};

#endif /* BITS_H */

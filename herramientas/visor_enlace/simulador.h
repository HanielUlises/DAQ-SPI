/*
 * Modelo del dsPIC y del motor para usar el visor sin hardware.
 *
 * Reproduce la lógica de enlace.c, igual que el modelo de
 * protocolo/prueba_protocolo.c: la respuesta de la transferencia k se
 * construye al terminar la k-1, el status se reporta una vez y se limpia, la
 * vigilancia expira tras 50 ms sin tramas válidas y la causa de un reinicio
 * sale en la primera respuesta que arma la interrupción.
 *
 * El motor es un sistema de primer orden: la velocidad del encoder sigue al
 * voltaje del DAC con ganancia kGanancia y constante de tiempo kTau. Se
 * pueden invertir bits al azar en ambas direcciones para ejercitar la
 * detección de errores.
 */
#ifndef SIMULADOR_H
#define SIMULADOR_H

#include <cstdint>
#include <cstring>
#include "../../protocolo/daq_protocolo.h"

class Simulador {
public:
    double ber = 0.0;           /* probabilidad de invertir cada bit, por dirección */

    Simulador() { arrancar(0u); }

    /* Reinicio del dsPIC; causa 0 equivale a un dsPIC que ya estaba en marcha */
    void arrancar(uint8_t causa)
    {
        seq_eco_ = 0u;
        status_ = 0u;
        dac_ = DAQ_DAC_CERO;
        en_falla_ = true;
        posicion_ = 0.0;
        velocidad_ = 0.0;
        construir_respuesta();
        status_ = (uint8_t)(causa << DAQ_STATUS_REINICIO_POS);
    }

    /* Transferencia completa en el instante t_us */
    void transferir(const uint8_t *mosi, uint8_t *miso, double t_us)
    {
        mover(t_us);
        if (!en_falla_ && t_us - t_valida_ > kVigilanciaUs) {
            en_falla_ = true;
            status_ |= DAQ_STATUS_VIGILANCIA;
            dac_ = DAQ_DAC_CERO;
        }

        uint8_t recibido[DAQ_TRAMA_LEN];
        memcpy(recibido, mosi, DAQ_TRAMA_LEN);
        memcpy(miso, tx_, DAQ_TRAMA_LEN);
        corromper(recibido);
        corromper(miso);

        if (recibido[DAQ_PC_INICIO] != DAQ_INICIO_PC) {
            status_ |= DAQ_STATUS_ERR_INICIO;
        } else if (daq_crc8(recibido, DAQ_TRAMA_LEN - 1u) != recibido[DAQ_PC_CRC]) {
            status_ |= DAQ_STATUS_ERR_CRC;
        } else {
            seq_eco_ = recibido[DAQ_PC_SEQ];
            if (recibido[DAQ_PC_FLAGS] & DAQ_FLAG_RESET_ENC) {
                posicion_ = 0.0;
            }
            dac_ = (recibido[DAQ_PC_FLAGS] & DAQ_FLAG_SALIDA_HAB)
                       ? (uint16_t)((recibido[DAQ_PC_DAC_H] << 8) | recibido[DAQ_PC_DAC_L])
                       : (uint16_t)DAQ_DAC_CERO;
            en_falla_ = false;
            t_valida_ = t_us;
        }
        construir_respuesta();
    }

    /* Duración simulada de SPI_ReadWrite: los bits a SCK más la latencia del
     * USB, con variaciones pequeñas y algún retraso largo ocasional */
    double duracion_us(double reloj_hz)
    {
        double d = 8.0 * DAQ_TRAMA_LEN * 1e6 / reloj_hz + 100.0 + 20.0 * uniforme();
        if (uniforme() < 0.001) {
            d += 300.0 + 1700.0 * uniforme();
        }
        return d;
    }

private:
    static constexpr double kGanancia = 4000.0;     /* cuentas/s por volt */
    static constexpr double kTau = 0.08;            /* s */
    static constexpr double kVigilanciaUs = 50000.0;

    uint8_t tx_[DAQ_TRAMA_LEN];
    uint8_t seq_eco_ = 0u;
    uint8_t status_ = 0u;
    uint16_t dac_ = DAQ_DAC_CERO;
    bool en_falla_ = true;
    double t_valida_ = 0.0;
    double t_anterior_ = 0.0;
    double posicion_ = 0.0;
    double velocidad_ = 0.0;
    uint64_t azar_ = 0x9E3779B97F4A7C15ull;

    void construir_respuesta()
    {
        const uint32_t pos = (uint32_t)(int32_t)posicion_;
        tx_[DAQ_PIC_INICIO]   = DAQ_INICIO_PIC;
        tx_[DAQ_PIC_SEQ]      = seq_eco_;
        tx_[DAQ_PIC_ENC0]     = (uint8_t)pos;
        tx_[DAQ_PIC_ENC0 + 1] = (uint8_t)(pos >> 8);
        tx_[DAQ_PIC_ENC0 + 2] = (uint8_t)(pos >> 16);
        tx_[DAQ_PIC_ENC0 + 3] = (uint8_t)(pos >> 24);
        tx_[DAQ_PIC_STATUS]   = status_;
        tx_[DAQ_PIC_CRC]      = daq_crc8(tx_, DAQ_TRAMA_LEN - 1u);
        status_ = 0u;
    }

    void mover(double t_us)
    {
        double dt = (t_us - t_anterior_) * 1e-6;
        t_anterior_ = t_us;
        if (dt <= 0.0 || dt > 0.1) return;
        const double v = dac_ / DAQ_DAC_ESCALA - DAQ_V_MAX;
        const double a = dt / kTau < 1.0 ? dt / kTau : 1.0;
        velocidad_ += (kGanancia * v - velocidad_) * a;
        posicion_ += velocidad_ * dt;
    }

    double uniforme()
    {
        azar_ ^= azar_ << 13;
        azar_ ^= azar_ >> 7;
        azar_ ^= azar_ << 17;
        return (double)(azar_ >> 11) * (1.0 / 9007199254740992.0);
    }

    void corromper(uint8_t *b)
    {
        if (ber <= 0.0) return;
        for (unsigned i = 0; i < DAQ_TRAMA_LEN; i++) {
            for (unsigned j = 0; j < 8; j++) {
                if (uniforme() < ber) b[i] ^= (uint8_t)(1u << j);
            }
        }
    }
};

#endif /* SIMULADOR_H */

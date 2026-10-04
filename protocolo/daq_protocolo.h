/*
 * Protocolo de comunicación PC <-> dsPIC (DAQ-SPI).
 *
 * Una sola transferencia SPI full-duplex de DAQ_TRAMA_LEN bytes por paso de
 * simulación, delimitada por CS. La respuesta que el dsPIC envía en la
 * transferencia k se construye al terminar la transferencia k-1.
 *
 * Este archivo lo comparten el firmware (XC-DSC) y el software de la PC
 * (MSVC/MinGW), por lo que se limita a C99 sin dependencias.
 *
 * Referencia: docs/propuesta.pdf, sección "Formato de trama".
 */
#ifndef DAQ_PROTOCOLO_H
#define DAQ_PROTOCOLO_H

#include <stdint.h>

#define DAQ_TRAMA_LEN           8u

/* Encabezados */
#define DAQ_INICIO_PC           0xA5u   /* PC -> dsPIC (MOSI) */
#define DAQ_INICIO_PIC          0x5Au   /* dsPIC -> PC (MISO) */

/* Trama PC -> dsPIC */
#define DAQ_PC_INICIO           0u
#define DAQ_PC_SEQ              1u
#define DAQ_PC_DAC_H            2u
#define DAQ_PC_DAC_L            3u
#define DAQ_PC_FLAGS            4u
#define DAQ_PC_CRC              7u      /* bytes 5 y 6 reservados (0) */

/* Trama dsPIC -> PC */
#define DAQ_PIC_INICIO          0u
#define DAQ_PIC_SEQ             1u      /* eco de la última trama válida */
#define DAQ_PIC_ENC0            2u      /* int32 con signo, little-endian */
#define DAQ_PIC_STATUS          6u
#define DAQ_PIC_CRC             7u

/* Byte flags (PC -> dsPIC) */
#define DAQ_FLAG_SALIDA_HAB     0x01u   /* 0: el DAC se lleva a 0 V */
#define DAQ_FLAG_RESET_ENC      0x02u   /* pone en cero el contador del QEI */

/* Byte status (dsPIC -> PC). Se reportan una vez y se limpian. */
#define DAQ_STATUS_ERR_CRC      0x01u
#define DAQ_STATUS_ERR_INICIO   0x02u
#define DAQ_STATUS_VIGILANCIA   0x04u   /* expiró el temporizador de vigilancia */
#define DAQ_STATUS_ERR_LONGITUD 0x08u   /* la trama no tuvo DAQ_TRAMA_LEN bytes */

/* Conversión de voltaje: valor = 13107 (V + 2.5), intervalo [-2.5, 2.5] V */
#define DAQ_DAC_CERO            32767u  /* 0 V */
#define DAQ_DAC_ESCALA          13107.0
#define DAQ_V_MAX               2.5

/* CRC-8, polinomio 0x07, valor inicial 0x00, sin reflexión (CRC-8/SMBUS).
 * Valor de verificación: daq_crc8("123456789", 9) = 0xF4.
 * Se usa tabla para acortar la interrupción de fin de trama en el dsPIC. */
static const uint8_t daq_crc8_tabla[256] = {
    0x00u, 0x07u, 0x0Eu, 0x09u, 0x1Cu, 0x1Bu, 0x12u, 0x15u,
    0x38u, 0x3Fu, 0x36u, 0x31u, 0x24u, 0x23u, 0x2Au, 0x2Du,
    0x70u, 0x77u, 0x7Eu, 0x79u, 0x6Cu, 0x6Bu, 0x62u, 0x65u,
    0x48u, 0x4Fu, 0x46u, 0x41u, 0x54u, 0x53u, 0x5Au, 0x5Du,
    0xE0u, 0xE7u, 0xEEu, 0xE9u, 0xFCu, 0xFBu, 0xF2u, 0xF5u,
    0xD8u, 0xDFu, 0xD6u, 0xD1u, 0xC4u, 0xC3u, 0xCAu, 0xCDu,
    0x90u, 0x97u, 0x9Eu, 0x99u, 0x8Cu, 0x8Bu, 0x82u, 0x85u,
    0xA8u, 0xAFu, 0xA6u, 0xA1u, 0xB4u, 0xB3u, 0xBAu, 0xBDu,
    0xC7u, 0xC0u, 0xC9u, 0xCEu, 0xDBu, 0xDCu, 0xD5u, 0xD2u,
    0xFFu, 0xF8u, 0xF1u, 0xF6u, 0xE3u, 0xE4u, 0xEDu, 0xEAu,
    0xB7u, 0xB0u, 0xB9u, 0xBEu, 0xABu, 0xACu, 0xA5u, 0xA2u,
    0x8Fu, 0x88u, 0x81u, 0x86u, 0x93u, 0x94u, 0x9Du, 0x9Au,
    0x27u, 0x20u, 0x29u, 0x2Eu, 0x3Bu, 0x3Cu, 0x35u, 0x32u,
    0x1Fu, 0x18u, 0x11u, 0x16u, 0x03u, 0x04u, 0x0Du, 0x0Au,
    0x57u, 0x50u, 0x59u, 0x5Eu, 0x4Bu, 0x4Cu, 0x45u, 0x42u,
    0x6Fu, 0x68u, 0x61u, 0x66u, 0x73u, 0x74u, 0x7Du, 0x7Au,
    0x89u, 0x8Eu, 0x87u, 0x80u, 0x95u, 0x92u, 0x9Bu, 0x9Cu,
    0xB1u, 0xB6u, 0xBFu, 0xB8u, 0xADu, 0xAAu, 0xA3u, 0xA4u,
    0xF9u, 0xFEu, 0xF7u, 0xF0u, 0xE5u, 0xE2u, 0xEBu, 0xECu,
    0xC1u, 0xC6u, 0xCFu, 0xC8u, 0xDDu, 0xDAu, 0xD3u, 0xD4u,
    0x69u, 0x6Eu, 0x67u, 0x60u, 0x75u, 0x72u, 0x7Bu, 0x7Cu,
    0x51u, 0x56u, 0x5Fu, 0x58u, 0x4Du, 0x4Au, 0x43u, 0x44u,
    0x19u, 0x1Eu, 0x17u, 0x10u, 0x05u, 0x02u, 0x0Bu, 0x0Cu,
    0x21u, 0x26u, 0x2Fu, 0x28u, 0x3Du, 0x3Au, 0x33u, 0x34u,
    0x4Eu, 0x49u, 0x40u, 0x47u, 0x52u, 0x55u, 0x5Cu, 0x5Bu,
    0x76u, 0x71u, 0x78u, 0x7Fu, 0x6Au, 0x6Du, 0x64u, 0x63u,
    0x3Eu, 0x39u, 0x30u, 0x37u, 0x22u, 0x25u, 0x2Cu, 0x2Bu,
    0x06u, 0x01u, 0x08u, 0x0Fu, 0x1Au, 0x1Du, 0x14u, 0x13u,
    0xAEu, 0xA9u, 0xA0u, 0xA7u, 0xB2u, 0xB5u, 0xBCu, 0xBBu,
    0x96u, 0x91u, 0x98u, 0x9Fu, 0x8Au, 0x8Du, 0x84u, 0x83u,
    0xDEu, 0xD9u, 0xD0u, 0xD7u, 0xC2u, 0xC5u, 0xCCu, 0xCBu,
    0xE6u, 0xE1u, 0xE8u, 0xEFu, 0xFAu, 0xFDu, 0xF4u, 0xF3u,
};

static inline uint8_t daq_crc8(const uint8_t *datos, uint8_t n)
{
    uint8_t crc = 0u;
    uint8_t i;

    for (i = 0u; i < n; i++) {
        crc = daq_crc8_tabla[crc ^ datos[i]];
    }
    return crc;
}

/* Convierte un voltaje al código del DAC. Satura a [-DAQ_V_MAX, DAQ_V_MAX]
 * y lleva NaN a 0 V. Sólo se usa en la PC. */
static inline uint16_t daq_voltaje_a_dac(double v)
{
    if (!(v == v)) {
        v = 0.0;
    }
    if (v > DAQ_V_MAX) {
        v = DAQ_V_MAX;
    }
    if (v < -DAQ_V_MAX) {
        v = -DAQ_V_MAX;
    }
    return (uint16_t)(DAQ_DAC_ESCALA * (v + DAQ_V_MAX));
}

/* Arma la trama que envía la PC. */
static inline void daq_armar_trama_pc(uint8_t trama[DAQ_TRAMA_LEN], uint8_t seq,
                                      uint16_t dac, uint8_t flags)
{
    trama[DAQ_PC_INICIO] = DAQ_INICIO_PC;
    trama[DAQ_PC_SEQ]    = seq;
    trama[DAQ_PC_DAC_H]  = (uint8_t)(dac >> 8);
    trama[DAQ_PC_DAC_L]  = (uint8_t)dac;
    trama[DAQ_PC_FLAGS]  = flags;
    trama[5]             = 0u;
    trama[6]             = 0u;
    trama[DAQ_PC_CRC]    = daq_crc8(trama, DAQ_TRAMA_LEN - 1u);
}

/* Verifica encabezado y CRC de la respuesta del dsPIC y extrae sus campos.
 * Devuelve 0 si la trama es válida. */
static inline int daq_leer_trama_pic(const uint8_t trama[DAQ_TRAMA_LEN], uint8_t *seq,
                                     int32_t *posicion, uint8_t *status)
{
    if (trama[DAQ_PIC_INICIO] != DAQ_INICIO_PIC) {
        return -1;
    }
    if (daq_crc8(trama, DAQ_TRAMA_LEN - 1u) != trama[DAQ_PIC_CRC]) {
        return -2;
    }
    *seq = trama[DAQ_PIC_SEQ];
    *posicion = (int32_t)((uint32_t)trama[DAQ_PIC_ENC0]
                        | ((uint32_t)trama[DAQ_PIC_ENC0 + 1u] << 8)
                        | ((uint32_t)trama[DAQ_PIC_ENC0 + 2u] << 16)
                        | ((uint32_t)trama[DAQ_PIC_ENC0 + 3u] << 24));
    *status = trama[DAQ_PIC_STATUS];
    return 0;
}

#endif /* DAQ_PROTOCOLO_H */

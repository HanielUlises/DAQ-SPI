#!/usr/bin/env python3
"""Convierte una captura PPM (P6) de visor_enlace a PNG, sin dependencias.

Uso: ppm_a_png.py captura.ppm captura.png
"""
import struct
import sys
import zlib


def leer_ppm(ruta):
    with open(ruta, 'rb') as f:
        datos = f.read()
    campos = []
    i = 0
    while len(campos) < 4:
        while datos[i:i + 1].isspace():
            i += 1
        if datos[i:i + 1] == b'#':
            i = datos.index(b'\n', i) + 1
            continue
        j = i
        while not datos[j:j + 1].isspace():
            j += 1
        campos.append(datos[i:j])
        i = j
    if campos[0] != b'P6' or int(campos[3]) != 255:
        raise SystemExit(f'{ruta}: se esperaba un PPM P6 de 8 bits')
    ancho, alto = int(campos[1]), int(campos[2])
    return ancho, alto, datos[i + 1:i + 1 + ancho * alto * 3]


def escribir_png(ruta, ancho, alto, pixeles):
    def bloque(tipo, contenido):
        return (struct.pack('>I', len(contenido)) + tipo + contenido
                + struct.pack('>I', zlib.crc32(tipo + contenido) & 0xFFFFFFFF))

    fila = ancho * 3
    crudo = b''.join(b'\x00' + pixeles[y * fila:(y + 1) * fila] for y in range(alto))
    with open(ruta, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(bloque(b'IHDR', struct.pack('>IIBBBBB', ancho, alto, 8, 2, 0, 0, 0)))
        f.write(bloque(b'IDAT', zlib.compress(crudo, 6)))
        f.write(bloque(b'IEND', b''))


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    escribir_png(sys.argv[2], *leer_ppm(sys.argv[1]))


if __name__ == '__main__':
    main()

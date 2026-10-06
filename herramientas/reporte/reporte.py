#!/usr/bin/env python3
"""Genera el reporte en PDF de una prueba del enlace.

Lee el CSV que escribe `prueba_enlace -registro`, calcula las estadísticas,
dibuja las gráficas con gnuplot (terminal cairolatex, para que el texto use la
tipografía del documento) y compila el reporte con LaTeX.

Uso:
    reporte.py prueba.csv [otra.csv ...]

Por cada `prueba.csv` se crea la carpeta `prueba/` con los archivos
intermedios y el reporte `prueba.pdf` junto al CSV. Si existe
`prueba_notas.tex`, se incluye como sección de observaciones, y si existe
`prueba.png` (captura de visor_enlace), como figura.

Requiere gnuplot (con cairolatex) y latexmk con pdflatex.
"""
import csv
import math
import os
import shutil
import statistics
import subprocess
import sys

TRAMA_LEN = 8
INICIO_PIC = 0x5A
STATUS = [
    (0x01, 'pic_crc', 'CRC (reportado por el dsPIC)'),
    (0x02, 'pic_inicio', 'Encabezado (reportado por el dsPIC)'),
    (0x08, 'pic_longitud', 'Longitud (reportado por el dsPIC)'),
    (0x04, 'pic_vigilancia', 'Vigilancia (reportado por el dsPIC)'),
    (0xF0, 'pic_reinicio', 'Reinicio (reportado por el dsPIC)'),
]
# Causa de reinicio en los 4 bits altos del status (DAQ_REINICIO_*)
CAUSAS_REINICIO = ['', 'encendido', 'bajo voltaje', 'MCLR', 'watchdog', 'reset por software',
                   'opcode ilegal / W sin inicializar', 'configuración', 'trampa: oscilador',
                   'trampa: pila', 'trampa: dirección', 'trampa: matemática', 'trampa: DMA',
                   'trampa: hard', 'trampa: otra', 'desconocida']
TIPOS_PC = [
    ('usb', 'USB / longitud'),
    ('eco', 'Eco distinto del envío'),
    ('inicio', 'Encabezado'),
    ('crc', 'CRC'),
    ('seq', 'Número de secuencia'),
]
PLANTILLA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'plantilla.tex')


def crc8(datos):
    """CRC-8/SMBUS, igual que daq_crc8() en protocolo/daq_protocolo.h."""
    c = 0
    for b in datos:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def respuesta(seq, pos, status):
    t = bytes([INICIO_PIC, seq]) + (pos & 0xFFFFFFFF).to_bytes(4, 'little') + bytes([status])
    return t + bytes([crc8(t)])


def distancia(a, b):
    return sum(bin(x ^ y).count('1') for x, y in zip(a, b))


def tex(texto):
    """Escapa texto para LaTeX."""
    reemplazos = {'\\': r'\textbackslash{}', '_': r'\_', '%': r'\%', '&': r'\&',
                  '#': r'\#', '$': r'\$', '{': r'\{', '}': r'\}', '~': r'\textasciitilde{}',
                  '^': r'\textasciicircum{}'}
    return ''.join(reemplazos.get(c, c) for c in texto)


def num(x, decimales=0):
    """Número con separador de miles (espacio fino)."""
    s = f'{x:,.{decimales}f}'
    return s.replace(',', r'\,')


def percentil(ordenados, p):
    if not ordenados:
        return float('nan')
    i = min(len(ordenados) - 1, max(0, int(math.ceil(p / 100.0 * len(ordenados))) - 1))
    return ordenados[i]


def leer(ruta):
    meta = {}
    filas = []
    with open(ruta, newline='') as f:
        lineas = []
        for linea in f:
            if linea.startswith('#'):
                clave, _, valor = linea[1:].partition(':')
                meta[clave.strip()] = valor.strip()
            else:
                lineas.append(linea)
    for r in csv.DictReader(lineas):
        filas.append({
            'k': int(r['trama']),
            't': float(r['t_us']),
            'dt': float(r['dt_us']),
            'mosi': bytes.fromhex(r['mosi']),
            'miso': bytes.fromhex(r['miso']),
            'res': r['resultado'],
            'status': int(r['status'], 16) if r['status'] else None,
        })
    return meta, filas


def configuracion(meta):
    args = meta.get('comando', '').split()
    conf = {'tramas': 100000, 'reloj': 1000000, 'periodo': 0.0, 'lazo': False, 'salida': False}
    for i, a in enumerate(args):
        sig = args[i + 1] if i + 1 < len(args) else ''
        if a == '-n':
            conf['tramas'] = int(sig)
        elif a == '-reloj':
            conf['reloj'] = int(sig)
        elif a == '-periodo':
            conf['periodo'] = float(sig)
        elif a == '-lazo':
            conf['lazo'] = True
        elif a == '-salida':
            conf['salida'] = True
    return conf


def es_arranque(filas, i):
    """Respuesta con secuencia 0 y posición 0 cuando la trama anterior no
    tenía secuencia 0: coincide con la que el firmware construye al arrancar."""
    return (i > 0 and filas[i]['miso'][:6] == bytes([INICIO_PIC, 0, 0, 0, 0, 0])
            and filas[i - 1]['mosi'][1] != 0)


def analizar(filas, conf):
    a = {}
    dts = sorted(f['dt'] for f in filas)
    a['n'] = len(filas)
    a['t_total'] = (filas[-1]['t'] + filas[-1]['dt']) * 1e-6 if filas else 0.0
    a['dt_min'] = dts[0]
    a['dt_media'] = statistics.fmean(dts)
    a['dt_mediana'] = statistics.median(dts)
    a['dt_p99'] = percentil(dts, 99)
    a['dt_p999'] = percentil(dts, 99.9)
    a['dt_max'] = dts[-1]
    a['dt_sobre_periodo'] = sum(1 for x in dts if conf['periodo'] > 0 and x > conf['periodo'])

    # Separación entre el fin de una transferencia y el inicio de la siguiente
    sep = sorted(filas[i + 1]['t'] - (filas[i]['t'] + filas[i]['dt']) for i in range(len(filas) - 1))
    a['sep_min'] = sep[0] if sep else float('nan')
    a['sep_mediana'] = statistics.median(sep) if sep else float('nan')

    # Conteo por tipo, en el mismo orden que prueba_enlace
    cuenta = {clave: 0 for clave, _ in TIPOS_PC}
    cuenta.update({clave: 0 for _, clave, _ in STATUS})
    causas = {}
    acumulado = []
    for f in filas:
        if f['res'] in cuenta:
            cuenta[f['res']] += 1
        if f['status'] is not None and f['res'] in ('ok', 'seq'):
            # Igual que status_contable(): la vigilancia en la trama 1 viene
            # de la inactividad previa a la prueba.
            status = f['status'] & ~0x04 if f['k'] == 1 else f['status']
            for bit, clave, _ in STATUS:
                if status & bit:
                    cuenta[clave] += 1
            if status >> 4:
                causas[status >> 4] = causas.get(status >> 4, 0) + 1
        acumulado.append(dict(cuenta))
    a['cuenta'] = cuenta
    a['causas'] = causas
    a['acumulado'] = acumulado
    a['errores'] = sum(cuenta.values())
    a['tramas_ok'] = sum(1 for f in filas if f['res'] == 'ok')

    # Mapa de bits erróneos: se compara cada respuesta con la esperada
    mapa = [[0] * 8 for _ in range(TRAMA_LEN)]
    analizadas = 0
    distintas = 0
    pos = 0
    for i, f in enumerate(filas):
        if conf['lazo']:
            esperada = f['mosi']
        else:
            if i == 0 or f['miso'][0] != INICIO_PIC or es_arranque(filas, i):
                continue
            if f['res'] in ('ok', 'seq'):
                pos = int.from_bytes(f['miso'][2:6], 'little', signed=True)
            # El dsPIC repite el número de secuencia de la última trama que
            # aceptó (normalmente la anterior) y el status puede traer una
            # bandera pendiente: se toma la candidata más cercana.
            candidatas = [respuesta(filas[j]['mosi'][1], pos, st)
                          for j in (i - 1, i - 2, i - 3) if j >= 0
                          for st in {0, 1, 2, 4, 8, 9, 10, 12, f['miso'][6]}]
            esperada = min(candidatas, key=lambda c: distancia(c, f['miso']))
        analizadas += 1
        if esperada != f['miso']:
            distintas += 1
            for byte in range(TRAMA_LEN):
                x = esperada[byte] ^ f['miso'][byte]
                for bit in range(8):
                    if (x >> bit) & 1:
                        mapa[byte][bit] += 1
    a['arranque'] = 0 if conf['lazo'] else sum(1 for i in range(1, len(filas)) if es_arranque(filas, i))
    a['mapa'] = mapa
    a['mapa_tramas'] = analizadas
    a['mapa_distintas'] = distintas
    total_bits = sum(map(sum, mapa))
    a['bits_erroneos'] = total_bits
    a['bits_msb'] = sum(mapa[b][7] for b in range(TRAMA_LEN))
    return a


def escribir_datos(dir_salida, filas, a):
    # Duración de cada transferencia: todos los valores altos y una muestra
    # del resto, para que la gráfica vectorial no pese demasiado.
    umbral = a['dt_p99']
    paso = max(1, len(filas) // 20000)
    with open(os.path.join(dir_salida, 'duracion.dat'), 'w') as f:
        for i, fila in enumerate(filas):
            if fila['dt'] >= umbral or i % paso == 0:
                f.write(f"{fila['t'] * 1e-6:.6f} {fila['dt']:.1f}\n")

    # Histograma con cubetas logarítmicas (20 por década)
    cubetas = {}
    for fila in filas:
        c = int(math.floor(20 * math.log10(max(fila['dt'], 1.0))))
        cubetas[c] = cubetas.get(c, 0) + 1
    with open(os.path.join(dir_salida, 'histograma.dat'), 'w') as f:
        for c in range(min(cubetas) - 1, max(cubetas) + 2):
            f.write(f'{10 ** (c / 20):.3f} {cubetas.get(c, 0)}\n')

    # Errores acumulados por tipo
    tipos = [(c, d) for c, d in TIPOS_PC + [(c, d) for _, c, d in STATUS] if a['cuenta'][c] > 0]
    paso = max(1, len(filas) // 2000)
    with open(os.path.join(dir_salida, 'acumulado.dat'), 'w') as f:
        for i in list(range(0, len(filas), paso)) + [len(filas) - 1]:
            f.write(f"{filas[i]['k']} " + ' '.join(str(a['acumulado'][i][c]) for c, _ in tipos) + '\n')

    # Mapa de bits: filas = byte 0..7, columnas = bit 7..0
    with open(os.path.join(dir_salida, 'bits.dat'), 'w') as f:
        for byte in range(TRAMA_LEN):
            f.write(' '.join(str(a['mapa'][byte][bit]) for bit in range(7, -1, -1)) + '\n')
    return tipos


GNUPLOT_COMUN = r"""
set terminal cairolatex pdf input colortext size 15cm,6.5cm font ",9"
set border 3 lw 1.5
set tics nomirror out
set grid lc rgb "#d0d0d0" lw 1
set key top left Left reverse samplen 2 box opaque
set lmargin 11
set ylabel offset -1.5
"""


def comando_corto(comando):
    """Comando sin directorios, para que quepa en la tabla."""
    return ' '.join(os.path.basename(x) if '/' in x or '\\' in x else x for x in comando.split())


def graficar(dir_salida, tipos, a, conf):
    scripts = {}
    periodo = conf['periodo']
    linea_periodo = (f'set arrow from graph 0, first {periodo} to graph 1, first {periodo} '
                     f'nohead dt 2 lw 2 lc rgb "#c0392b" front\n'
                     f'set label "periodo" at graph 0.99, first {periodo * 1.15} right tc rgb "#c0392b" front\n'
                     if periodo > 0 else '')

    scripts['fig_duracion'] = GNUPLOT_COMUN + f"""
set output 'fig_duracion.tex'
set xlabel 'Tiempo desde el inicio [s]'
set ylabel 'Duración [$\\mu$s]'
set logscale y
set yrange [{max(1.0, a['dt_min'] * 0.7)}:{max(a['dt_max'], periodo) * 1.6}]
unset key
{linea_periodo}
plot 'duracion.dat' using 1:2 with points pt 7 ps 0.12 lc rgb "#2c6fbb"
"""

    linea_periodo_x = (f'set arrow from first {periodo}, graph 0 to first {periodo}, graph 1 '
                       f'nohead dt 2 lw 2 lc rgb "#c0392b" front\n' if periodo > 0 else '')
    scripts['fig_histograma'] = GNUPLOT_COMUN + f"""
set output 'fig_histograma.tex'
set xlabel 'Duración de SPI\\_ReadWrite [$\\mu$s]'
set ylabel 'Transferencias'
set logscale xy
set yrange [0.7:*]
unset key
{linea_periodo_x}
plot 'histograma.dat' using 1:($2 > 0 ? $2 : 0.01) with fsteps lw 2 lc rgb "#2c6fbb"
"""

    colores = ['#2c6fbb', '#c0392b', '#27ae60', '#8e44ad', '#e67e22', '#16a085', '#7f8c8d', '#d35400', '#34495e']
    if tipos:
        curvas = ', '.join(f"'acumulado.dat' using ($1/1000):{i + 2} with lines lw 2 lc rgb '{colores[i % len(colores)]}' "
                           f"title '{tex(d)}'" for i, (_, d) in enumerate(tipos))
        scripts['fig_acumulado'] = GNUPLOT_COMUN + f"""
set output 'fig_acumulado.tex'
set xlabel 'Trama (miles)'
set ylabel 'Errores acumulados'
set key top left
plot {curvas}
"""

    if a['bits_erroneos'] > 0:
        scripts['fig_bits'] = GNUPLOT_COMUN.replace('size 15cm,6.5cm', 'size 13cm,8cm') + r"""
set output 'fig_bits.tex'
unset grid
unset key
set border 15 lw 1
set xlabel 'Bit (MSB a la izquierda)'
set ylabel 'Byte de la trama'
set xtics ('7' 0, '6' 1, '5' 2, '4' 3, '3' 4, '2' 5, '1' 6, '0' 7) scale 0
set ytics 0, 1, 7 scale 0
set yrange [7.5:-0.5]
set xrange [-0.5:7.5]
set logscale cb
set cblabel 'Bits erróneos' offset 2.5
set rmargin 12
set palette defined (0 "#fff5eb", 1 "#fd8d3c", 2 "#7f2704")
plot 'bits.dat' matrix using 1:2:($3 > 0 ? $3 : NaN) with image, \
     '' matrix using 1:2:($3 >= 10000 ? sprintf('%.0fk', $3 / 1000) : $3 > 0 ? sprintf('%d', $3) : '') with labels
"""

    for nombre, guion in scripts.items():
        ruta = os.path.join(dir_salida, nombre + '.gp')
        with open(ruta, 'w') as f:
            f.write(guion)
        subprocess.run(['gnuplot', nombre + '.gp'], cwd=dir_salida, check=True)
    return set(scripts)


def describir(conf):
    modo = 'lazo interno del FT2232H (sin dsPIC)' if conf['lazo'] else 'FT2232H con el dsPIC (firmware dsPicDaq)'
    ritmo = f"cada {num(conf['periodo'])} $\\mu$s ({num(1e6 / conf['periodo'])} tramas/s)" \
        if conf['periodo'] > 0 else 'lo más rápido posible'
    return modo, ritmo


def observaciones(a, conf):
    obs = []
    n = a['n']
    if a['errores'] == 0:
        obs.append(f"Las {num(n)} tramas se transfirieron sin errores.")
    else:
        obs.append(f"Se registraron {num(a['errores'])} eventos de error en {num(n)} tramas; "
                   f"{num(a['tramas_ok'])} respuestas ({100.0 * a['tramas_ok'] / n:.1f}\\,\\%) fueron válidas "
                   f"y con el número de secuencia esperado.")
    if a['bits_erroneos'] > 0:
        obs.append(f"De {num(a['mapa_tramas'])} respuestas comparadas con la esperada, "
                   f"{num(a['mapa_distintas'])} difieren en {num(a['bits_erroneos'])} bits. "
                   f"El {100.0 * a['bits_msb'] / a['bits_erroneos']:.1f}\\,\\% de los bits erróneos "
                   f"corresponde al bit más significativo de algún byte.")
    if a['causas']:
        obs.append('El dsPIC reportó reinicios por: ' + ', '.join(
            f"{tex(CAUSAS_REINICIO[c])} ({num(n)})" for c, n in sorted(a['causas'].items())) + '.')
    if a['arranque'] > 0:
        obs.append(f"{num(a['arranque'])} respuestas ({100.0 * a['arranque'] / n:.1f}\\,\\%) traen número de "
                   f"secuencia 0 y posición 0, como la que construye el firmware al arrancar, aunque la trama "
                   f"anterior tenía otra secuencia.")
    if conf['periodo'] > 0:
        obs.append(f"{num(a['dt_sobre_periodo'])} transferencias ({100.0 * a['dt_sobre_periodo'] / n:.2f}\\,\\%) "
                   f"duraron más que el periodo; en Simulink cada una sería un paso atrasado.")
    else:
        obs.append(f"La separación mediana entre el fin de una transferencia y el inicio de la siguiente "
                   f"fue de {a['sep_mediana']:.0f}\\,$\\mu$s (mínima {a['sep_min']:.0f}\\,$\\mu$s), medida en la PC.")
    return obs


def generar(ruta_csv):
    meta, filas = leer(ruta_csv)
    if not filas:
        raise SystemExit(f'{ruta_csv}: sin tramas')
    conf = configuracion(meta)
    base = os.path.splitext(ruta_csv)[0]
    nombre = os.path.basename(base)
    dir_salida = base
    os.makedirs(dir_salida, exist_ok=True)

    a = analizar(filas, conf)
    tipos = escribir_datos(dir_salida, filas, a)
    figuras = graficar(dir_salida, tipos, a, conf)
    modo, ritmo = describir(conf)

    filas_errores = '\n'.join(
        f"{tex(d)} & {num(a['cuenta'][c])} & {100.0 * a['cuenta'][c] / a['n']:.2f} \\\\"
        for c, d in TIPOS_PC + [(c, d) for _, c, d in STATUS]
        if not (conf['lazo'] and c not in ('usb', 'eco')) and (conf['lazo'] or c != 'eco'))

    notas = base + '_notas.tex'
    secciones_figuras = []
    if os.path.exists(base + '.png'):
        shutil.copy(base + '.png', os.path.join(dir_salida, 'captura.png'))
        secciones_figuras.append(
            r'\begin{figure}[H]\centering'
            r'\includegraphics[width=\linewidth]{captura.png}'
            r'\caption{Ventana de \code{visor\_enlace} al terminar la prueba.}'
            r'\end{figure}')
    if 'fig_duracion' in figuras:
        secciones_figuras.append(
            r'\begin{figure}[H]\centering\input{fig_duracion.tex}'
            r'\caption{Duración de cada llamada a \code{SPI\_ReadWrite} a lo largo de la prueba'
            + (r'; la línea discontinua marca el periodo' if conf['periodo'] > 0 else '') + '.}'
            r'\end{figure}')
    if 'fig_histograma' in figuras:
        secciones_figuras.append(
            r'\begin{figure}[H]\centering\input{fig_histograma.tex}'
            r'\caption{Distribución de la duración de las transferencias (20 cubetas por década).}'
            r'\end{figure}')
    if 'fig_acumulado' in figuras:
        secciones_figuras.append(
            r'\begin{figure}[H]\centering\input{fig_acumulado.tex}'
            r'\caption{Errores acumulados por tipo a lo largo de la prueba.}'
            r'\end{figure}')
    if 'fig_bits' in figuras:
        referencia = 'la trama enviada' if conf['lazo'] else \
            'la respuesta esperada (eco de la secuencia anterior, posición del encoder y status pendiente)'
        secciones_figuras.append(
            r'\begin{figure}[H]\centering\input{fig_bits.tex}'
            r'\caption{Bits erróneos por posición dentro de la trama, comparando cada respuesta '
            r'con encabezado correcto contra ' + referencia + '.'
            + ('' if conf['lazo'] else ' No se incluyen las respuestas de arranque, que se cuentan aparte.') + '}'
            r'\end{figure}')

    with open(PLANTILLA) as f:
        documento = f.read()
    reemplazos = {
        'TITULO': tex(nombre.replace('_', ' ')),
        'FECHA': tex(meta.get('fecha', '')),
        'PLATAFORMA': tex(meta.get('plataforma', '')),
        'COMANDO': tex(comando_corto(meta.get('comando', ''))),
        'FILA_VISOR': f"Visor        & {tex(meta['visor'])} \\\\" if 'visor' in meta else '',
        'MODO': modo,
        'RITMO': ritmo,
        'RELOJ': num(conf['reloj'] / 1000),
        'SALIDA': 'rampa de $\\pm$1\\,V' if conf['salida'] else 'deshabilitada (0\\,V)',
        'TRAMAS': num(a['n']),
        'DURACION_TOTAL': f"{a['t_total']:.2f}",
        'TASA': num(a['n'] / a['t_total']) if a['t_total'] > 0 else '--',
        'RESULTADO': r'\textbf{sin errores}' if a['errores'] == 0 else r'\textbf{con errores}',
        'FILAS_ERRORES': filas_errores,
        'DT_MIN': f"{a['dt_min']:.0f}",
        'DT_MEDIA': f"{a['dt_media']:.0f}",
        'DT_MEDIANA': f"{a['dt_mediana']:.0f}",
        'DT_P99': f"{a['dt_p99']:.0f}",
        'DT_P999': f"{a['dt_p999']:.0f}",
        'DT_MAX': f"{a['dt_max']:.0f}",
        'OBSERVACIONES': '\n'.join(r'\item ' + o for o in observaciones(a, conf)),
        'NOTAS': r'\input{' + os.path.abspath(notas) + '}' if os.path.exists(notas) else '',
        'FIGURAS': '\n\n'.join(secciones_figuras),
    }
    for clave, valor in reemplazos.items():
        documento = documento.replace('@@' + clave + '@@', valor)
    with open(os.path.join(dir_salida, 'reporte.tex'), 'w') as f:
        f.write(documento)

    r = subprocess.run(['latexmk', '-pdf', '-interaction=nonstopmode', '-halt-on-error', 'reporte.tex'],
                       cwd=dir_salida, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stdout[-3000:])
        raise SystemExit(f'{ruta_csv}: falló la compilación de LaTeX')
    shutil.copy(os.path.join(dir_salida, 'reporte.pdf'), base + '.pdf')
    print(f'{base}.pdf')


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for ruta in sys.argv[1:]:
        generar(ruta)


if __name__ == '__main__':
    main()

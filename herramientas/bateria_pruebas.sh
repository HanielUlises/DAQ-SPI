#!/bin/bash
# Batería de pruebas del enlace con la tarjeta conectada: lazo interno del
# FT2232H y dsPIC, cada uno lo más rápido posible y a 1 kHz. Guarda el
# registro CSV y la salida de cada prueba y genera los reportes en PDF.
#
# Uso: herramientas/bateria_pruebas.sh [-visor] [-n tramas] [dir_salida]
#   -visor   corre las pruebas con visor_enlace (y guarda una captura de la
#            ventana al terminar cada una) en lugar de prueba_enlace
#   -n       tramas por prueba (100000 por omisión)
# Por omisión los resultados van a resultados/<fecha>_<herramienta>/.
#
# La salida analógica queda en 0 V en todas las pruebas.
set -uo pipefail

RAIZ=$(cd "$(dirname "$0")/.." && pwd)
VISOR=0
TRAMAS=100000
SALIDA=""
while [ $# -gt 0 ]; do
    case $1 in
        -visor) VISOR=1 ;;
        -n) TRAMAS=$2; shift ;;
        -*) echo "Uso: $0 [-visor] [-n tramas] [dir_salida]" >&2; exit 2 ;;
        *) SALIDA=$1 ;;
    esac
    shift
done

if [ $VISOR -eq 1 ]; then
    HERRAMIENTA=visor_enlace
    make -s -C "$RAIZ/herramientas/visor_enlace" || exit 2
    PROGRAMA=$RAIZ/herramientas/visor_enlace/visor_enlace
else
    HERRAMIENTA=prueba_enlace
    make -s -C "$RAIZ/herramientas/prueba_enlace" || exit 2
    PROGRAMA=$RAIZ/herramientas/prueba_enlace/prueba_enlace
fi
SALIDA=${SALIDA:-$RAIZ/resultados/$(date +%F)_$HERRAMIENTA}
mkdir -p "$SALIDA"

# nombre y opciones de cada prueba
PRUEBAS=(
    "lazo_max|-lazo"
    "lazo_1khz|-lazo -periodo 1000"
    "dspic_max|"
    "dspic_1khz|-periodo 1000"
)

resumen=()
for p in "${PRUEBAS[@]}"; do
    nombre=${p%%|*}
    opciones=${p#*|}
    base=$SALIDA/$nombre
    extra=()
    if [ $VISOR -eq 1 ]; then
        extra=(-iniciar -salir -captura "$base.ppm")
    fi
    echo "== $nombre: $HERRAMIENTA -n $TRAMAS $opciones"
    # shellcheck disable=SC2086
    "$PROGRAMA" -n "$TRAMAS" $opciones -registro "$base.csv" "${extra[@]}" > "$base.txt" 2>&1
    codigo=$?
    case $codigo in
        0) estado="sin errores" ;;
        1) estado="con errores" ;;
        *) estado="no se ejecutó (código $codigo)"; cat "$base.txt" ;;
    esac
    resumen+=("$nombre: $estado")
    if [ -f "$base.ppm" ]; then
        python3 "$RAIZ/herramientas/reporte/ppm_a_png.py" "$base.ppm" "$base.png" && rm -f "$base.ppm"
    fi
    if [ $codigo -le 1 ] && [ -s "$base.csv" ]; then
        python3 "$RAIZ/herramientas/reporte/reporte.py" "$base.csv" > /dev/null || echo "   falló el reporte de $nombre"
    fi
done

echo
printf '%s\n' "${resumen[@]}"
echo "Resultados en $SALIDA"

#!/bin/bash
# Compila dsPicDaq con XC-DSC sin MPLAB X (Linux) y deja el .hex en
# MPLAB/dsPicDaq.hex. Usa las mismas opciones que el proyecto de MPLAB X
# (-O1, pila de 16 bytes con guarda).
#
# Variables de entorno:
#   XC   carpeta bin de XC-DSC  (por omisión /opt/microchip/xc-dsc/v4.00/bin)
#   DFP  carpeta xc16 del pack dsPIC33CK-MC_DFP
#        (por omisión ~/.mchp_packs/Microchip/dsPIC33CK-MC_DFP/1.9.370/xc16)
set -euo pipefail

XC=${XC:-/opt/microchip/xc-dsc/v4.00/bin}
DFP=${DFP:-$HOME/.mchp_packs/Microchip/dsPIC33CK-MC_DFP/1.9.370/xc16}
DIR=$(cd "$(dirname "$0")" && pwd)
OUT=$DIR/build/linux
HEX=$DIR/../dsPicDaq.hex

cd "$DIR"
rm -rf "$OUT"
mkdir -p "$OUT"

COMUN=(-mcpu=33CK64MC105 "-mdfp=$DFP" -omf=elf)
FUENTES=(main.c enlace.c dac.c reloj.c reinicio.c
         mcc_generated_files/*/src/*.c mcc_generated_files/*/src/*.s)

OBJS=()
for f in "${FUENTES[@]}"; do
    o=$OUT/$(echo "$f" | tr / _).o
    case $f in
        *.c) "$XC/xc-dsc-gcc" "${COMUN[@]}" -c -O1 -msmart-io=1 -Wall -msfr-warn=off "$f" -o "$o" ;;
        *.s) "$XC/xc-dsc-gcc" "${COMUN[@]}" -c "$f" -o "$o" ;;
    esac
    OBJS+=("$o")
done

"$XC/xc-dsc-gcc" "${COMUN[@]}" -o "$OUT/dsPicDaq.elf" "${OBJS[@]}" \
    "-Wl,--script=$DFP/support/dsPIC33C/gld/p33CK64MC105.gld,--local-stack,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map=$OUT/dsPicDaq.map,--report-mem" \
    | grep -E 'Total "(program|data)"'
"$XC/xc-dsc-bin2hex" "$OUT/dsPicDaq.elf" -a "-mdfp=$DFP"
cp "$OUT/dsPicDaq.hex" "$HEX"
echo "Listo: $(realpath "$HEX")"

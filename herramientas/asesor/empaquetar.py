#!/usr/bin/env python3
"""
Arma el paquete para el asesor a partir de su carpeta "Archivos Finales"
(los modelos MergedPic.slx, ReadPicModificado.slx y WritePicModificado.slx
con sus S-Functions), sin cambiar sus modelos ni renombrar nada:

  - copia ORIGEN en DESTINO (ORIGEN no se modifica);
  - en cada carpeta de SIMULINK/ cuyo modelo use initializePic, escribirPic o
    leerPic, mueve los .cpp y .mexw64 anteriores de esos bloques a
    anteriores/ y deja en su lugar:
      * los .cpp autónomos, generados desde SIMULINK/DaqPic con
        daq_bloques.h, daq_protocolo.h y daq_canal.h incluidos, que compilan
        en esa carpeta con "mex initializePic.cpp libmpsse.lib", etc.;
      * los .mexw64 de --mex, si se da (compilados para Windows);
      * abrir_modelo.m y compilar_bloques.m (herramientas/asesor);
      * "Abrir <modelo>.bat", que abre MATLAB con esa carpeta como carpeta
        actual y el modelo abierto;
  - pone otro "Abrir <modelo>.bat" en la raíz del paquete, el firmware en
    FIRMWARE/, su proyecto de MPLAB X en MPLAB/dsPicDaq.X (con
    protocolo/daq_protocolo.h, que incluye como ../../protocolo) y un
    LEEME.txt.

Uso: empaquetar.py ORIGEN DESTINO [--mex CARPETA_CON_MEXW64]
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import zipfile

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
DAQPIC = os.path.join(RAIZ, 'SIMULINK', 'DaqPic')
ASESOR = os.path.dirname(os.path.abspath(__file__))

BLOQUES = ['initializePic', 'escribirPic', 'leerPic']
ARGUMENTOS_MEX = {'initializePic': 'initializePic.cpp libmpsse.lib',
                  'escribirPic': 'escribirPic.cpp', 'leerPic': 'leerPic.cpp'}

# Encabezados del proyecto que se incluyen dentro de cada .cpp autónomo
LOCALES = {
    'daq_bloques.h': os.path.join(DAQPIC, 'daq_bloques.h'),
    '../../protocolo/daq_protocolo.h': os.path.join(RAIZ, 'protocolo', 'daq_protocolo.h'),
    '../../protocolo/daq_canal.h': os.path.join(RAIZ, 'protocolo', 'daq_canal.h'),
    'daqPicInicio.cpp': os.path.join(DAQPIC, 'daqPicInicio.cpp'),
    'daqPicEscribir.cpp': os.path.join(DAQPIC, 'daqPicEscribir.cpp'),
    'daqPicLeer.cpp': os.path.join(DAQPIC, 'daqPicLeer.cpp'),
}

# Proyecto del firmware: sólo los archivos versionados (sin build/ ni dist/)
PROYECTO_FIRMWARE = ['MPLAB/dsPicDaq.X', 'protocolo/daq_protocolo.h']

FIRMWARE = {
    'dsPicDaq.hex': os.path.join(RAIZ, 'MPLAB', 'dsPicDaq.hex'),
    'dsPicController_anterior.hex': os.path.join(
        RAIZ, 'MPLAB', 'dsPicController.X', 'dist', 'default', 'production',
        'dsPicController.X.production.hex'),
}

BAT = r'''@echo off
rem Abre {modelo}.slx en MATLAB con su carpeta como carpeta actual, para que
rem Simulink use los initializePic, escribirPic y leerPic de esa carpeta. Si
rem no estan compilados para este MATLAB, los compila (abrir_modelo.m).
rem Funciona desde cualquier ubicacion: la ruta es relativa a este archivo.
chcp 65001 >nul
setlocal
set "MODELO={modelo}"
set "CARPETA={carpeta}"
if "%CARPETA:~-1%"=="\" set "CARPETA=%CARPETA:~0,-1%"
if not exist "%CARPETA%\%MODELO%.slx" (
    echo No se encontro "%CARPETA%\%MODELO%.slx".
    echo Conserve la estructura de carpetas del paquete.
    pause
    exit /b 1
)
if not exist "%CARPETA%\abrir_modelo.m" (
    echo Falta abrir_modelo.m en "%CARPETA%".
    pause
    exit /b 1
)

rem MATLAB: el de la ruta del sistema o la version mas reciente instalada
set "MATLAB="
for /f "delims=" %%i in ('where matlab.exe 2^>nul') do if not defined MATLAB set "MATLAB=%%i"
if not defined MATLAB for /d %%d in ("%ProgramFiles%\MATLAB\R*") do if exist "%%d\bin\matlab.exe" set "MATLAB=%%d\bin\matlab.exe"
if not defined MATLAB (
    echo No se encontro MATLAB. Abralo a mano, cambie la carpeta actual a
    echo   %CARPETA%
    echo y ejecute: abrir_modelo('%MODELO%'^)
    pause
    exit /b 1
)

echo Abriendo %MODELO% desde "%CARPETA%"
echo con "%MATLAB%"
start "" "%MATLAB%" -sd "%CARPETA%" -r "abrir_modelo('%MODELO%')"
'''

LEEME = '''DAQ-SPI: bloques initializePic, escribirPic y leerPic nuevos
=============================================================

Esta carpeta es una copia de "Archivos Finales" con los bloques nuevos ya
puestos en las carpetas de los modelos de siempre. Los modelos (.slx) son los
mismos, sin cambios; sólo cambian los bloques que usan:

{modelos}
Los bloques anteriores de cada carpeta quedaron en su subcarpeta anteriores/.

Cómo abrir un modelo
--------------------
Doble clic en "Abrir MergedPic.bat" (o en el del modelo que se quiera), en
esta carpeta o dentro de la carpeta del modelo. Abre MATLAB con la carpeta
del modelo como carpeta actual, verifica que los bloques sean los de esa
carpeta (si no lo son o no cargan con esa versión de MATLAB, los compila) y
abre el modelo. Después: Run.

La carpeta puede moverse a cualquier lugar; hay que conservar su estructura.

Sin el .bat: en MATLAB, cambiar la carpeta actual a la del modelo y ejecutar
    abrir_modelo('MergedPic')

Para recompilar los bloques (requiere mex -setup C++), en la carpeta del
modelo:
    compilar_bloques

Qué hacen los bloques
---------------------
    initializePic   sin parámetros     salida: llave
    escribirPic     entradas: 1) voltaje [V] (±2.5 V), 2) llave
    leerPic         entrada: llave     salida: posición del encoder [cuentas]

Sólo initializePic usa el FT2232H: en cada paso hace una sola transferencia
SPI que envía el voltaje de escribirPic y recibe la posición que entrega
leerPic, con un retardo de dos periodos. Al terminar la simulación la salida
queda en 0 V y la ventana de comandos muestra una línea como:
    initializePic: 10000 pasos de 0.001 s, 0 errores de comunicacion, 0 pasos atrasados
Ambos contadores deben quedar en 0.

Requisitos
----------
1. Firmware dsPicDaq en la Curiosity Nano: copiar FIRMWARE/dsPicDaq.hex a la
   unidad CURIOSITY. Los bloques nuevos NO funcionan con el firmware
   anterior (para regresar a él: FIRMWARE/dsPicController_anterior.hex).
2. Driver de FTDI instalado (carpeta DRIVER FTDI).
3. Cable de GND entre la Curiosity Nano y la tarjeta del FT2232H.
4. El modelo debe usar un solver de paso fijo (los incluidos usan 0.001 s).
{mex}
Código del firmware
-------------------
MPLAB/dsPicDaq.X es el proyecto de MPLAB X de FIRMWARE/dsPicDaq.hex
(dsPIC33CK64MC105, XC16 o XC-DSC, pack dsPIC33CK-MC_DFP). Incluye
../../protocolo/daq_protocolo.h, así que debe quedar junto a la carpeta
protocolo/ de este paquete. En MPLAB X: File > Open Project > dsPicDaq.X y
"Make and Program Device" con la Curiosity Nano conectada.

No regenerar con MCC (botón Generate): config_bits.c está modificado a mano
para cambiar al PLL y reinicio.c sustituye el manejo de trampas de MCC.
'''


def expandir(ruta, vistos):
    salida = []
    with open(ruta, encoding='utf-8') as f:
        for linea in f:
            m = re.match(r'\s*#include\s+"([^"]+)"', linea)
            if m and m.group(1) in LOCALES:
                dep = LOCALES[m.group(1)]
                if dep not in vistos:
                    vistos.add(dep)
                    salida.append(f'/* ---- inicio de {m.group(1)} ---- */\n')
                    salida.append(expandir(dep, vistos))
                    salida.append(f'/* ---- fin de {m.group(1)} ---- */\n')
            else:
                salida.append(linea)
    return ''.join(salida)


def cpp_autonomo(bloque):
    encabezado = (
        '/*\n'
        f' * Archivo autónomo generado por herramientas/asesor/empaquetar.py a partir de\n'
        f' * SIMULINK/DaqPic/{bloque}.cpp del repositorio DAQ-SPI, con daq_bloques.h,\n'
        ' * daq_protocolo.h y daq_canal.h incluidos. Compila en la carpeta del modelo:\n'
        ' *\n'
        f' *     mex {ARGUMENTOS_MEX[bloque]}\n'
        ' *\n'
        ' * Requiere ftd2xx.h, libmpsse_spi.h y libmpsse.lib en la misma carpeta.\n'
        ' * No editar: los cambios van en SIMULINK/DaqPic.\n'
        ' */\n')
    return encabezado + expandir(os.path.join(DAQPIC, bloque + '.cpp'), set())


def bloques_del_modelo(slx):
    """S-Functions de BLOQUES que usa el modelo"""
    usados = set()
    with zipfile.ZipFile(slx) as z:
        for nombre in z.namelist():
            if nombre.endswith('.xml'):
                texto = z.read(nombre).decode('utf-8', 'replace')
                usados.update(re.findall(r'<P Name="FunctionName">([^<]+)</P>', texto))
    return [b for b in BLOQUES if b in usados]


def escribir(ruta, texto, crlf=True):
    with open(ruta, 'w', encoding='utf-8', newline='\r\n' if crlf else '\n') as f:
        f.write(texto)


def main():
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument('origen', help='carpeta "Archivos Finales" del asesor')
    p.add_argument('destino', help='carpeta nueva del paquete (no debe existir)')
    p.add_argument('--mex', help='carpeta con initializePic.mexw64, escribirPic.mexw64 y leerPic.mexw64')
    a = p.parse_args()

    if os.path.exists(a.destino):
        sys.exit(f'{a.destino} ya existe; elija otra carpeta o bórrela.')
    if not os.path.isdir(os.path.join(a.origen, 'SIMULINK')):
        sys.exit(f'{a.origen} no tiene la carpeta SIMULINK.')
    if a.mex:
        for b in BLOQUES:
            if not os.path.isfile(os.path.join(a.mex, b + '.mexw64')):
                sys.exit(f'Falta {b}.mexw64 en {a.mex}.')

    shutil.copytree(a.origen, a.destino)
    fuentes = {b: cpp_autonomo(b) for b in BLOQUES}
    modelos = []
    simulink = os.path.join(a.destino, 'SIMULINK')
    for carpeta in sorted(os.listdir(simulink)):
        c = os.path.join(simulink, carpeta)
        slx = os.path.join(c, carpeta + '.slx')
        if not os.path.isfile(slx):
            continue
        usados = bloques_del_modelo(slx)
        if not usados:
            continue
        anteriores = os.path.join(c, 'anteriores')
        os.makedirs(anteriores, exist_ok=True)
        for b in usados:
            for ext in ('.cpp', '.mexw64'):
                viejo = os.path.join(c, b + ext)
                if os.path.exists(viejo):
                    shutil.move(viejo, os.path.join(anteriores, b + ext))
            escribir(os.path.join(c, b + '.cpp'), fuentes[b])
            if a.mex:
                shutil.copy2(os.path.join(a.mex, b + '.mexw64'), c)
        for m in ('abrir_modelo.m', 'compilar_bloques.m'):
            shutil.copy2(os.path.join(ASESOR, m), c)
        escribir(os.path.join(c, f'Abrir {carpeta}.bat'), BAT.format(modelo=carpeta, carpeta=r'%~dp0'))
        escribir(os.path.join(a.destino, f'Abrir {carpeta}.bat'),
                 BAT.format(modelo=carpeta, carpeta=r'%~dp0SIMULINK\%MODELO%'))
        modelos.append((carpeta, usados))
        print(f'{carpeta}: {", ".join(usados)}')

    os.makedirs(os.path.join(a.destino, 'FIRMWARE'), exist_ok=True)
    for nombre, ruta in FIRMWARE.items():
        shutil.copy2(ruta, os.path.join(a.destino, 'FIRMWARE', nombre))

    archivos = subprocess.run(['git', '-C', RAIZ, 'ls-files', '-z', '--', *PROYECTO_FIRMWARE],
                              check=True, capture_output=True).stdout.decode().split('\0')
    for rel in filter(None, archivos):
        destino = os.path.join(a.destino, *rel.split('/'))
        os.makedirs(os.path.dirname(destino), exist_ok=True)
        shutil.copy2(os.path.join(RAIZ, rel), destino)

    lista = ''.join(f'    SIMULINK/{m}/{m}.slx   {", ".join(u)}\n' for m, u in modelos)
    mex = ('5. Los .mexw64 vienen compilados para Windows de 64 bits. Si no cargan\n'
           '   con la versión de MATLAB instalada, el .bat los recompila (requiere\n'
           '   mex -setup C++).\n' if a.mex else
           '5. Los .mexw64 no vienen compilados: el .bat los compila la primera vez\n'
           '   (requiere mex -setup C++).\n')
    escribir(os.path.join(a.destino, 'LEEME.txt'), LEEME.format(modelos=lista, mex=mex))
    print(f'Listo: {a.destino}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Arma el paquete para el asesor a partir de su carpeta "Archivos Finales"
(los modelos MergedPic.slx, ReadPicModificado.slx y WritePicModificado.slx
con sus S-Functions), con la misma estructura (DRIVER FTDI, MPLAB, SIMULINK)
y sin cambiar sus modelos ni renombrar nada:

  - copia ORIGEN en DESTINO (ORIGEN no se modifica);
  - en cada carpeta de SIMULINK/ cuyo modelo use initializePic, escribirPic o
    leerPic, mueve los .cpp y .mexw64 anteriores de esos bloques y su
    ftdi_compartido.h a anteriores/ y deja en su lugar:
      * los .cpp y ftdi_compartido.h de SIMULINK/Asesor (los del asesor,
        adaptados a la trama del firmware dsPicDaq) con daq_protocolo.h y
        daq_canal.h junto a ellos, que compilan en esa carpeta con
        "mex initializePic.cpp libmpsse.lib -lfixedpoint", etc.;
      * los .mexw64 de --mex, si se da (compilados para Windows);
      * abrir_modelo.m y compilar_bloques.m (herramientas/asesor);
      * "Abrir <modelo>.bat", que abre MATLAB con esa carpeta como carpeta
        actual y el modelo abierto;
  - agrega el proyecto de MPLAB X del firmware nuevo, MPLAB/dsPicDaq.X, junto
    a los anteriores: autónomo (daq_protocolo.h dentro del proyecto, para que
    compile donde se copie) y con MPLAB/dsPicDaq.hex en dist/default/
    production, donde lo deja MPLAB X;
  - escribe un LEEME.txt.

Con --completo agrega además, como los paquetes anteriores, otro "Abrir
<modelo>.bat" en la raíz, el firmware en FIRMWARE/ y el resto del proyecto
con la estructura del repositorio: el protocolo, el código fuente de los
bloques (SIMULINK/Asesor y SIMULINK/DaqPic), las herramientas de prueba, los
documentos y los resultados.

Del repositorio sólo se copian los archivos versionados (sin build/ ni dist/).

Uso: empaquetar.py ORIGEN DESTINO [--mex CARPETA_CON_MEXW64] [--completo]
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import zipfile

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
ASESOR = os.path.dirname(os.path.abspath(__file__))

# Bloques del asesor adaptados al firmware nuevo y lo que comparten; en el
# paquete los encabezados del protocolo quedan junto a los .cpp
BLOQUES = ['initializePic', 'escribirPic', 'leerPic']
BLOQUES_ASESOR = os.path.join(RAIZ, 'SIMULINK', 'Asesor')
COMPARTIDOS = {
    'ftdi_compartido.h': os.path.join(BLOQUES_ASESOR, 'ftdi_compartido.h'),
    'daq_protocolo.h': os.path.join(RAIZ, 'protocolo', 'daq_protocolo.h'),
    'daq_canal.h': os.path.join(RAIZ, 'protocolo', 'daq_canal.h'),
}
RUTA_PROTOCOLO = b'../../protocolo/'

# Proyecto de MPLAB X del firmware nuevo, sin compilar.sh (sólo Linux); el
# protocolo se copia dentro del proyecto
FIRMWARE_X = 'MPLAB/dsPicDaq.X'
HEX = os.path.join(RAIZ, 'MPLAB', 'dsPicDaq.hex')

# Con --completo: resto del proyecto y firmware en FIRMWARE/
PROYECTO = ['protocolo', 'SIMULINK/Asesor', 'SIMULINK/DaqPic', 'herramientas', 'docs',
            'resultados', 'README.md', 'PRUEBAS.md']

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
El firmware nuevo es el proyecto MPLAB/dsPicDaq.X; los demás proyectos de
MPLAB/ son los anteriores, sin cambios.
{contenido}
Cómo abrir un modelo
--------------------
Doble clic en "Abrir <modelo>.bat"{donde}. Abre MATLAB con la carpeta
del modelo como carpeta actual, verifica que los bloques sean los de esa
carpeta (si no lo son o no cargan con esa versión de MATLAB, los compila) y
abre el modelo. Después: Run.

La carpeta puede moverse a cualquier lugar; hay que conservar su estructura.

Sin el .bat: en MATLAB, cambiar la carpeta actual a la del modelo y ejecutar
    abrir_modelo('WritePicModificado')

Para recompilar los bloques (requiere mex -setup C++), en la carpeta del
modelo:
    compilar_bloques

Qué hacen los bloques
---------------------
    initializePic   sin parámetros     salida: llave
    escribirPic     entradas: 1) voltaje [V] (±2.5 V), 2) llave
    leerPic         entrada: llave     salida: posición del encoder [cuentas]

Son los bloques de siempre: los tres comparten el FT2232H con la llave de
initializePic (ftdi_compartido.h). Lo que cambia es la trama del firmware
dsPicDaq: cada transferencia SPI son 8 bytes que llevan el voltaje al dsPIC
y traen la posición del encoder, con encabezado y CRC. escribirPic y leerPic
hacen una transferencia por paso; escribirPic envía aunque el voltaje no
cambie, porque si pasan más de 50 ms sin tramas el dsPIC lleva la salida a
0 V. initializePic abre la interfaz del FT2232H por la que responde el dsPIC,
hace que la simulación avance al ritmo del reloj de la PC y, al terminar,
deja la salida en 0 V y muestra en la ventana de comandos una línea como:
    initializePic: 3002 tramas, 0 errores de comunicacion
El contador de errores debe quedar en 0.

Requisitos
----------
1. Firmware dsPicDaq en la Curiosity Nano, compilado desde MPLAB X (ver
   "Firmware" abajo). Los bloques nuevos NO funcionan con el firmware
   anterior.
2. Driver de FTDI instalado (carpeta DRIVER FTDI).
3. Cable de GND entre la Curiosity Nano y la tarjeta del FT2232H.
4. El modelo debe usar un solver de paso fijo menor que 0.05 s (los
   incluidos usan 0.001 s y 0.02 s).
{mex}
Firmware
--------
MPLAB/dsPicDaq.X es el proyecto de MPLAB X del firmware nuevo, junto a los
proyectos anteriores. Se compila igual que ellos (dsPIC33CK64MC105, XC16
v2.10 o XC-DSC, pack dsPIC33CK-MC_DFP) y es autónomo: trae dentro
daq_protocolo.h, el formato de la trama que comparte con los bloques.

Para compilarlo y cargarlo:
  1. MPLAB X: File > Open Project > MPLAB/dsPicDaq.X.
  2. Conectar la Curiosity Nano por USB.
  3. "Make and Program Device" (o Clean and Build y después programar).

No regenerar con MCC (botón Generate): config_bits.c está modificado a mano
para cambiar al PLL y reinicio.c sustituye el manejo de trampas de MCC.

Para regresar al firmware anterior, abrir MPLAB/dsPicController.X y hacer
lo mismo.

Sin MPLAB X, la Curiosity Nano también se programa copiando el .hex ya
compilado a la unidad CURIOSITY que aparece al conectarla:
    nuevo:    {hex}
    anterior: {hex_anterior}
'''

# Sección "Contenido" del LEEME con --completo
CONTENIDO_COMPLETO = '''
Contenido
---------
    Abrir <modelo>.bat     abre el modelo con sus bloques (ver abajo)
    SIMULINK/              los modelos de siempre, con los bloques nuevos
    SIMULINK/Asesor/       código fuente de initializePic, escribirPic y
                           leerPic: los del asesor, adaptados a la trama nueva
    SIMULINK/DaqPic/       bloques daqPic (un bloque) y daqPicInicio/Escribir/
                           Leer (tres bloques); compilar.m los compila
    MPLAB/dsPicDaq.X/      proyecto de MPLAB X del firmware nuevo
    MPLAB/dsPic*.X         proyectos anteriores, sin cambios
    FIRMWARE/              dsPicDaq.hex y el firmware anterior
    protocolo/             formato de la trama, compartido por el firmware y
                           la PC, y su prueba sin hardware
    herramientas/          pruebas del enlace sin MATLAB (prueba_enlace, con
                           .exe para Windows; visor_enlace; arnes_simulink;
                           reporte) y el armado de este paquete (asesor/)
    docs/                  propuesta, informe, arquitectura y reporte del
                           mutex, en PDF y con su fuente en LaTeX
    resultados/            registros y reportes de las pruebas
    README.md, PRUEBAS.md  descripción del proyecto y procedimiento de pruebas
    DRIVER FTDI/           driver D2XX de FTDI
'''


def copiar_sin_ruta(origen, destino):
    """Copia un archivo del repositorio con los includes de ../../protocolo/
    cambiados a la misma carpeta, donde queda la copia del protocolo"""
    with open(origen, 'rb') as f:
        datos = f.read().replace(RUTA_PROTOCOLO, b'')
    os.makedirs(os.path.dirname(destino), exist_ok=True)
    with open(destino, 'wb') as f:
        f.write(datos)


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


def versionados(rutas):
    """Archivos versionados del repositorio bajo rutas (sin build/ ni dist/)"""
    salida = subprocess.run(['git', '-C', RAIZ, 'ls-files', '-z', '--', *rutas],
                            check=True, capture_output=True).stdout.decode()
    return [r for r in salida.split('\0') if r]


def copiar_firmware(destino):
    """MPLAB/dsPicDaq.X autónomo, con daq_protocolo.h dentro y el .hex en dist/"""
    proyecto = os.path.join(destino, *FIRMWARE_X.split('/'))
    for rel in versionados([FIRMWARE_X]):
        if not rel.endswith('/compilar.sh'):
            copiar_sin_ruta(os.path.join(RAIZ, rel), os.path.join(destino, *rel.split('/')))
    shutil.copy2(os.path.join(RAIZ, 'protocolo', 'daq_protocolo.h'), proyecto)
    produccion = os.path.join(proyecto, 'dist', 'default', 'production')
    os.makedirs(produccion)
    shutil.copy2(HEX, os.path.join(produccion, 'dsPicDaq.X.production.hex'))


def main():
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument('origen', help='carpeta "Archivos Finales" del asesor')
    p.add_argument('destino', help='carpeta nueva del paquete (no debe existir)')
    p.add_argument('--mex', help='carpeta con initializePic.mexw64, escribirPic.mexw64 y leerPic.mexw64')
    p.add_argument('--completo', action='store_true',
                   help='agrega el resto del proyecto, FIRMWARE/ y los .bat en la raíz')
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
        anteriores_bloques = [b + ext for b in usados for ext in ('.cpp', '.mexw64')]
        for nombre in anteriores_bloques + ['ftdi_compartido.h']:
            viejo = os.path.join(c, nombre)
            if os.path.exists(viejo):
                shutil.move(viejo, os.path.join(anteriores, nombre))
        for b in usados:
            copiar_sin_ruta(os.path.join(BLOQUES_ASESOR, b + '.cpp'), os.path.join(c, b + '.cpp'))
            if a.mex:
                shutil.copy2(os.path.join(a.mex, b + '.mexw64'), c)
        for nombre, ruta in COMPARTIDOS.items():
            copiar_sin_ruta(ruta, os.path.join(c, nombre))
        for m in ('abrir_modelo.m', 'compilar_bloques.m'):
            shutil.copy2(os.path.join(ASESOR, m), c)
        escribir(os.path.join(c, f'Abrir {carpeta}.bat'), BAT.format(modelo=carpeta, carpeta=r'%~dp0'))
        if a.completo:
            escribir(os.path.join(a.destino, f'Abrir {carpeta}.bat'),
                     BAT.format(modelo=carpeta, carpeta=r'%~dp0SIMULINK\%MODELO%'))
        modelos.append((carpeta, usados))
        print(f'{carpeta}: {", ".join(usados)}')

    copiar_firmware(a.destino)

    if a.completo:
        os.makedirs(os.path.join(a.destino, 'FIRMWARE'), exist_ok=True)
        for nombre, ruta in FIRMWARE.items():
            shutil.copy2(ruta, os.path.join(a.destino, 'FIRMWARE', nombre))
        for rel in versionados(PROYECTO):
            destino = os.path.join(a.destino, *rel.split('/'))
            os.makedirs(os.path.dirname(destino), exist_ok=True)
            shutil.copy2(os.path.join(RAIZ, rel), destino)
        textos = dict(contenido=CONTENIDO_COMPLETO,
                      donde=', en esta carpeta o dentro\nde la carpeta del modelo',
                      hex='FIRMWARE/dsPicDaq.hex',
                      hex_anterior='FIRMWARE/dsPicController_anterior.hex')
    else:
        textos = dict(contenido='',
                      donde=', dentro de la carpeta del modelo\n(por ejemplo SIMULINK/WritePicModificado)',
                      hex='MPLAB/dsPicDaq.X/dist/default/production/dsPicDaq.X.production.hex',
                      hex_anterior='MPLAB/dsPicController.X/dist/default/production/'
                                   'dsPicController.X.production.hex')

    lista = ''.join(f'    SIMULINK/{m}/{m}.slx   {", ".join(u)}\n' for m, u in modelos)
    mex = ('5. Los .mexw64 vienen compilados para Windows de 64 bits. Si no cargan\n'
           '   con la versión de MATLAB instalada, el .bat los recompila (requiere\n'
           '   mex -setup C++).\n' if a.mex else
           '5. Los .mexw64 no vienen compilados: el .bat los compila la primera vez\n'
           '   (requiere mex -setup C++).\n')
    escribir(os.path.join(a.destino, 'LEEME.txt'), LEEME.format(modelos=lista, mex=mex, **textos))
    print(f'Listo: {a.destino}')


if __name__ == '__main__':
    main()

# Pruebas en la tarjeta (Windows 11)

Procedimiento para validar en el hardware el firmware `dsPicDaq`, el programa
de prueba y el bloque `daqPic`, conforme al plan de validación de
[`docs/propuesta.pdf`](docs/propuesta.pdf). Los pasos deben ejecutarse en
orden: cada uno supone que el anterior terminó sin errores.

## Requisitos

- Driver CDM de FTDI instalado (`DRIVER FTDI/`). En el Administrador de
  dispositivos deben aparecer dos *USB Serial Converter* al conectar el
  FT2232H.
- MATLAB/Simulink con un compilador de C++ configurado para MEX (el mismo con
  el que se compilaron las S-Functions actuales).
- Conexiones según la tabla del [README](README.md#conexiones), incluido el
  cable de GND entre la Curiosity Nano y la tarjeta del FT2232H. Sin ese
  cable el enlace depende de cómo se conecten los USB: con cada tarjeta en un
  puerto distinto de la PC, alrededor del 0.7 % de las transferencias falla.
- Recomendado: analizador lógico con cinco canales y osciloscopio.

**Seguridad.** En los pasos 1 y 2 la salida analógica permanece en 0 V. El
paso 3 genera una rampa de ±1 V y el paso 4 una senoidal de 0.5 V, por lo que
el motor se mueve. En cualquier paso, si la PC deja de enviar tramas durante
50 ms, el dsPIC lleva el DAC a 0 V.

## 0. Prueba previa del FT2232H en Linux (opcional)

`prueba_enlace` también compila en Linux con libftdi1 (Fedora:
`libftdi-devel`). El FT2232H de la tarjeta tiene la EEPROM reprogramada como
`2099:0001` ("Custom DAC USB Bridge"), por lo que `ftdi_sio` no lo reclama y
hace falta una regla udev para usarlo sin `sudo` (una sola vez):

```
sudo cp herramientas/prueba_enlace/70-daq-spi.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

```
make -C herramientas/prueba_enlace
herramientas/prueba_enlace/prueba_enlace -lazo
herramientas/prueba_enlace/prueba_enlace -lazo -periodo 1000
```

`-lazo` une MOSI con MISO dentro del FT2232H, así que no requiere el dsPIC:
verifica que el FT2232H opere en modo MPSSE y que cada byte regrese intacto,
y mide la duración de las transferencias. Con el dsPIC conectado, las mismas
opciones de los pasos 2 y 3 funcionan igual que en Windows. Los tiempos
medidos en Linux no sustituyen a los de Windows, porque el driver es otro.

En Linux la Curiosity Nano también se monta como la unidad `CURIOSITY`, por
lo que el firmware se carga igual que en el paso 1, y
`MPLAB/dsPicDaq.X/compilar.sh` lo compila con XC-DSC sin MPLAB X.

## 1. Cargar el firmware

1. Conectar la Curiosity Nano por USB. Aparece la unidad `CURIOSITY`.
2. Copiar [`MPLAB/dsPicDaq.hex`](MPLAB/dsPicDaq.hex) a esa unidad y esperar a
   que termine la programación.

Como alternativa, abrir `MPLAB/dsPicDaq.X` en MPLAB X y ejecutar
*Make and Program Device*.

La terminal RD10 (LED0) se mantiene en alto durante la interrupción de fin de
trama: sin comunicación permanece en bajo y, con tramas a 1 kHz, el LED cambia
de intensidad.

## 2. Prueba del enlace (sin Simulink)

Cerrar en MATLAB cualquier modelo que utilice el FT2232H. En una terminal:

```
cd herramientas\prueba_enlace
prueba_enlace.exe
prueba_enlace.exe -periodo 1000
```

La primera ejecución envía 100 000 tramas tan rápido como lo permite el USB;
la segunda, a 1 kHz, como lo haría Simulink. Ambas deben terminar con
`Resultado: SIN ERRORES`. El programa imprime además la duración mínima, media
y máxima de cada transferencia; conviene registrarla, porque determina el
periodo de muestreo mínimo alcanzable.

### Interpretación de errores

| Síntoma | Causa probable |
|---|---|
| Todas las respuestas con error de encabezado o CRC | Firmware anterior cargado, MISO desconectado o el DMA de transmisión no entrega los bytes |
| Errores de número de secuencia aislados | El dsPIC no terminó de procesar una trama antes de la siguiente |
| El dsPIC reporta errores de longitud | `CS` no delimita tramas de 8 bytes (revisar AD3 → RB10) |
| CRC incorrecto en la PC y, en la respuesta siguiente, longitud incorrecta en el dsPIC | Pulsos falsos en `CS` por falta del cable de GND entre las tarjetas |
| El dsPIC reporta errores de CRC | Ruido en SCK o MOSI |
| El dsPIC reporta vigilancia | Pausas de más de 50 ms en la PC. La que trae la segunda respuesta viene de la inactividad previa a la prueba y no se cuenta |

### Verificación con analizador lógico

Canales: `CS` (RB10), SCK (RB13), MOSI (RB14), MISO (RB11) y RD10. Verificar:

1. Cada intervalo con `CS` en bajo contiene exactamente 8 bytes.
2. MOSI comienza con `A5` y MISO con `5A` en todas las tramas.
3. El segundo byte de MISO es igual al segundo byte de MOSI de la trama
   anterior. Esto confirma que el DMA entrega el segundo byte de la respuesta.
   `prueba_enlace` ya lo comprueba en cada trama (errores de número de
   secuencia): en `resultados/2026-10-06_dock` no hubo ninguno en 200 000
   tramas, por lo que este punto no requiere el analizador.
4. El pulso en RD10 inicia al subir `CS`. Su ancho es la duración de la
   interrupción de fin de trama (se estima en alrededor de 100 µs) y debe ser
   menor que el intervalo mínimo entre tramas.

## 3. Rampa en la salida analógica

```
prueba_enlace.exe -n 10000 -salida
```

Durante unos 10 s se envía una rampa triangular de ±1 V con periodo de 2000
tramas. En el osciloscopio, sobre `VOUTA` (terminal 1 del DAC8554), debe
observarse una señal triangular simétrica alrededor del nivel que corresponde a
0 V. Al terminar, la salida regresa a ese nivel.

## 4. Simulink

En MATLAB:

```matlab
cd SIMULINK/DaqPic
compilar                % genera daqPic, daqPicInicio, daqPicEscribir, daqPicLeer,
                        % initializePic, escribirPic y leerPic
crear_modelo            % genera y abre DaqPicPrueba.slx (un bloque)
crear_modelo_bloques    % genera y abre DaqPicBloquesPrueba.slx (tres bloques)
```

Ejecutar el modelo (10 s). La posición debe seguir a la senoidal aplicada, y
los indicadores `Errores` y `Atrasos` deben permanecer en 0.

Para usar el bloque en otro modelo, copiar el bloque `daqPic` de
`DaqPicPrueba.slx`. Sus parámetros son el periodo de muestreo y la
sincronización con tiempo real; su entrada es el voltaje y sus salidas son la
posición, los errores de comunicación y los pasos atrasados. No requiere
`initializePic` y no debe combinarse con los bloques anteriores en el mismo
modelo. La posición se entrega con un retardo de dos periodos respecto al
voltaje aplicado, lo que debe considerarse en el diseño del controlador.

`DaqPicBloquesPrueba.slx` es el mismo modelo con `daqPicInicio`,
`daqPicEscribir` y `daqPicLeer` (ver el README); se ejecuta igual y sus
indicadores también deben permanecer en 0.

Los modelos anteriores (`MergedPic.slx`, `ReadPicModificado.slx`,
`WritePicModificado.slx`) funcionan con el firmware nuevo sin cambios a través
de `initializePic`, `escribirPic` y `leerPic`, siempre que `SIMULINK/DaqPic` sea
la carpeta actual (si no, MATLAB toma los `.mexw64` anteriores de la carpeta
del modelo). Al terminar, la ventana de comandos debe mostrar 0 errores y 0
pasos atrasados.

Antes de pasar a Windows, los bloques pueden probarse en Linux con
`make -C herramientas/arnes_simulink correr`.

## Regresar al firmware anterior

Copiar `MPLAB/dsPicController.X/dist/default/production/dsPicController.X.production.hex`
a la unidad `CURIOSITY`. Los bloques anteriores compilados en `MergedPic`,
`ReadPicModificado` y `WritePicModificado` sólo funcionan con ese firmware.

## Resultados a registrar

- Salida completa de `prueba_enlace.exe` en ambas ejecuciones del paso 2.
- Capturas del analizador lógico de al menos dos tramas consecutivas y el
  ancho del pulso en RD10.
- Captura del osciloscopio del paso 3.
- Valores de `Errores` y `Atrasos` al terminar el paso 4.

### Reporte de cada prueba

Con `-registro archivo.csv`, `prueba_enlace` guarda una línea por trama
(tiempos, bytes enviados y recibidos, resultado). `herramientas/reporte`
genera a partir de ese archivo un reporte en PDF con tablas y gráficas
(duración de las transferencias, errores acumulados y mapa de bits
erróneos). Requiere Python 3, gnuplot y LaTeX (`latexmk`, `pdflatex`).

```
prueba_enlace -periodo 1000 -registro dspic_1khz.csv
python3 herramientas/reporte/reporte.py dspic_1khz.csv
```

Si junto al CSV existe `dspic_1khz_notas.tex`, se incluye en el reporte como
sección de análisis. `-reloj Hz` cambia la frecuencia de SCK (1 MHz por
omisión).

### Visor en tiempo real

`herramientas/visor_enlace` corre la misma prueba que `prueba_enlace` y, en
cada paso, un modelo equivalente al de Simulink con `daqPic`, de modo que
los experimentos con el motor pueden hacerse en Linux sin MATLAB:

- **Lazo abierto:** fuente → saturación → `daqPic`. La fuente es un
  generador de escalón, rampa, senoidal o cuadrada, en volts.
- **Lazo cerrado:** referencia → Σ → PID → saturación → `daqPic`, con la
  posición del encoder como retroalimentación. El PID es el *Discrete PID
  Controller* de Simulink en forma paralela (integrador de Euler hacia
  adelante, derivada con filtro N y anti-windup por sujeción). Como en
  `daqPic`, el paso k usa la posición que llegó en la transferencia k-1.

La ventana tiene una barra de herramientas (Iniciar/Detener con Ctrl+T,
modelo, periodo de muestreo `Ts`, tiempo final o número de tramas, salida),
el diagrama de bloques y el Scope. Un clic sobre un bloque muestra sus
parámetros a la izquierda; las ganancias, la referencia y la saturación se
cambian con la prueba en curso. La posición es relativa al inicio de la
prueba y se expresa en cuentas, vueltas, grados o radianes (bloque `daqPic`,
con las cuentas por vuelta del encoder). El Scope grafica el voltaje `u`, la
posición `y` con la referencia `r`, el error y la duración de cada
transferencia con el eje de tiempo ligado, con cursores, disparo por error y
estadísticas de la ventana; desde el bloque Scope se exportan `t`, `u`, `y`,
`r`, `e` y `dt` a `.mat` (MAT 4, `load` en MATLAB) o `.csv`, como *To
Workspace*. Se conservan los contadores `Errores` y `Atrasos`, la tabla de
tramas con error (con la respuesta esperada y el status de la respuesta
siguiente) y el mapa de bits erróneos.

La salida arranca deshabilitada (0 V) salvo con `-salida`; al habilitarla,
el generador y el PID arrancan desde t = 0. Requiere GLFW y OpenGL (Fedora:
`glfw-devel mesa-libGL-devel`) además de libftdi1.

```
make -C herramientas/visor_enlace
herramientas/visor_enlace/visor_enlace -periodo 1000
herramientas/visor_enlace/visor_enlace -simulado     # sin hardware
herramientas/visor_enlace/visor_enlace -periodo 1000 -tf 10 -cerrado -kp 0.003
make -C herramientas/visor_enlace prueba             # pruebas sin hardware
```

Las pruebas sin hardware cubren `bits.h`, el generador, el PID, la
exportación y el lazo cerrado completo contra el simulador del dsPIC y de un
motor de primer orden.

### Batería de pruebas

`herramientas/bateria_pruebas.sh` corre en secuencia el lazo interno y el
dsPIC, cada uno lo más rápido posible y a 1 kHz, guarda registro y salida de
cada prueba y genera los reportes. Con `-visor` usa el visor y agrega al
reporte la captura de la ventana al terminar.

```
herramientas/bateria_pruebas.sh -visor
```

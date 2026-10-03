# DAQ-SPI

Tarjeta de adquisición de datos de bajo costo, funcionalmente equivalente a
una tarjeta tipo Quanser, basada en el microcontrolador dsPIC33CK64MC105
(Curiosity Nano). El sistema se controla en tiempo real desde Simulink mediante
un puente USB–SPI FT2232H y ofrece una salida analógica de ±2.5 V y la lectura
de un encoder en cuadratura.

## Arquitectura

![Arquitectura del sistema](docs/arquitectura.png)

En la PC, el modelo de Simulink invoca S-Functions en C++ que utilizan la
biblioteca libMPSSE-SPI de FTDI para operar el FT2232H como maestro SPI. El
dsPIC actúa como esclavo de este bus: recibe el valor que debe escribirse en el
DAC y devuelve la posición del encoder. Internamente, el dsPIC emplea tres
periféricos:

| Periférico | Función |
|---|---|
| SPI1 (esclavo) | Enlace con el FT2232H |
| SPI2 (maestro) | Escritura al DAC8554 |
| QEI1 | Conteo de pulsos del encoder del motor |

El reloj del microcontrolador proviene del oscilador interno FRC de 8 MHz, sin
PLL (F<sub>osc</sub> = 8 MHz, F<sub>cy</sub> = 4 MHz).

### Convertidor digital-analógico

La salida analógica la genera un DAC8554 de Texas Instruments, convertidor de
cuatro canales y 16 bits con interfaz serial, en encapsulado TSSOP-16 montado
sobre un adaptador a DIP PA0017C de Proto-Advantage (SSOP-16, paso de 0.65 mm). El dsPIC actúa como maestro de este dispositivo
a través de SPI2, que es un enlace unidireccional: sólo se utilizan las líneas
de reloj (`SCLK`), datos (`DIN`) y sincronía (`SYNC`).

Cada actualización consiste en una trama de 24 bits enmarcada por `SYNC` en
nivel bajo: un byte de control seguido del código de 16 bits recibido desde la
PC. El byte de control `0x10` selecciona la dirección 0 (`A1 = A0 = 0`), el
canal A y el modo de actualización inmediata de un solo canal, por lo que la
salida utilizada es `VOUTA`. Los canales B, C y D no se emplean.

## Conexiones

| Señal | dsPIC | FT2232H | DAC8554 |
|---|---|---|---|
| SCK1 (entrada) | RB13 | AD0 (SCK) | |
| SDI1 | RB14 | AD1 (MOSI) | |
| SDO1 | RB11 | AD2 (MISO) | |
| SS1 | RB10 | AD3 (CS) | |
| SCK2 | RB0 | | pin 10 |
| SDO2 | RB2 | | pin 11 |
| SYNC (GPIO) | RB4 | | pin 9 |
| QEI A / B | RB8 / RB9 | | |
| LED0 | RD10 | | |

## Protocolo de comunicación

El FT2232H opera a 1 MHz en modo SPI 0, con `CS` activo en bajo en `DBUS3` y
un *latency timer* de 1 ms. En la versión actual, la escritura y la lectura se
realizan como transacciones independientes.

### Escritura (PC → dsPIC)

Cada transacción consta de tres bytes:

| Byte | Contenido |
|---|---|
| 0 | Encabezado `0xAA` |
| 1 | Valor del DAC, byte alto |
| 2 | Valor del DAC, byte bajo |

El voltaje solicitado se convierte según `valor = 13107 · (V + 2.5)`, de modo
que −2.5 V, 0 V y +2.5 V corresponden a 0, 32767 y 65535, respectivamente. El
dsPIC retransmite el valor al DAC precedido del byte de comando `0x10`.

### Lectura (dsPIC → PC)

La PC lee doce bytes y localiza en ellos la secuencia siguiente:

| Byte | Contenido |
|---|---|
| 0–1 | Encabezado `0xAA 0x55` |
| 2–5 | Posición del QEI, `int32` *little-endian* |

## Firmware

El firmware se desarrolló en MPLAB X con MCC y el compilador XC-DSC (o XC16),
usando el pack `dsPIC33CK-MC_DFP`. Se mantienen tres variantes: una que sólo
transmite la posición del encoder, otra que sólo recibe y escribe al DAC, y
`dsPicController`, que integra ambas funciones y es la que se utiliza en
operación normal. `dsPicDaq` implementa el protocolo nuevo descrito en
[Trabajo en curso](#trabajo-en-curso) y se encuentra en validación.

La tarjeta se programa con el depurador integrado de la Curiosity Nano o
copiando el archivo `.hex` a la unidad USB `CURIOSITY` que aparece al
conectarla.

## Software en la PC

El acceso al FT2232H se divide en tres S-Functions que comparten un mismo
*handle*:

| S-Function | Función |
|---|---|
| `initializePic` | Abre el canal SPI y entrega como señal (`uint64`) un apuntador a la estructura `HandleFTDI` |
| `escribirPic` | Recibe un voltaje en el intervalo ±2.5 V y lo transmite al dsPIC |
| `leerPic` | Obtiene la posición del encoder |

`HandleFTDI` incluye un mutex del sistema operativo que impide el acceso
simultáneo al dispositivo desde distintos bloques.

Se incluyen los binarios `.mexw64` para Windows de 64 bits. Para recompilarlos
se ejecuta, desde MATLAB y en el directorio del modelo, una instrucción de la
forma:

```matlab
mex escribirPic.cpp libmpsse.lib
```

Las bibliotecas `libmpsse.dll` y `msvcr120.dll` deben encontrarse en el mismo
directorio que el modelo, y es necesario instalar el driver CDM de FTDI
(v2.12.36.4) incluido en el repositorio.

## Limitaciones conocidas

Cuando la lectura y la escritura operan simultáneamente, la comunicación se
corrompe. El análisis del firmware y del uso del bus identifica las causas
siguientes:

1. **Uso half-duplex de un bus full-duplex.** La PC ejecuta `SPI_Write` y
   `SPI_Read` por separado. Durante la escritura, el dsPIC desplaza por MISO
   datos del encoder que la PC descarta; durante la lectura, la PC envía bytes
   de relleno que el dsPIC interpreta como datos.
2. **Respuesta sin sincronía con el maestro.** El esclavo escribe la posición
   en la FIFO de transmisión cada vez que hay espacio, sin relación con el
   momento en que la PC lee. Los datos llegan desfasados y desalineados.
3. **Espera activa en el lazo principal.** La escritura en SPI1 bloquea
   mientras la FIFO de transmisión esté llena, condición que sólo cambia cuando
   el maestro genera reloj. Durante ese tiempo no se atiende la recepción.
4. **Desbordamiento sin recuperación.** Si la FIFO de recepción se llena, se
   activa `SPIROV`; el driver generado por MCC no limpia la bandera y el módulo
   deja de recibir.
5. **Falta de delimitación de tramas.** El dsPIC no utiliza `CS` para
   identificar el inicio de cada trama y se limita a buscar `0xAA`, valor que
   puede aparecer dentro de los datos.

El mutex de la PC evita el acceso concurrente al FT2232H, pero no interviene en
ninguno de estos fenómenos. Adicionalmente, al terminar la simulación se envía
el valor `0x0000`, que corresponde a −2.5 V y no a 0 V, por lo que el motor
permanece energizado.

## Trabajo en curso

Se sustituye el protocolo actual por una única transferencia full-duplex de
ocho bytes por paso de simulación (`SPI_ReadWrite`), delimitada por `CS` y
protegida con número de secuencia y CRC-8. El análisis, el formato de trama y
el plan de validación se encuentran en [`docs/propuesta.pdf`](docs/propuesta.pdf).

| Etapa | Estado |
|---|---|
| 1. Firmware `dsPicDaq` | Implementado; compila sin advertencias con XC-DSC v4.00. Pendiente de validar en la tarjeta |
| 2. Programa de prueba en la PC | Implementado (`herramientas/prueba_enlace`). Pendiente de ejecutar con el hardware |
| 3. Bloque de Simulink | Implementado (`SIMULINK/DaqPic`). Pendiente de compilar y ejecutar en MATLAB |
| 4. Pruebas con el motor | Pendiente |

El procedimiento de prueba en la tarjeta se describe en [`PRUEBAS.md`](PRUEBAS.md).
Se incluyen el firmware compilado (`MPLAB/dsPicDaq.hex`) y el programa de
prueba compilado para Windows de 64 bits.

El formato de trama está definido en un solo lugar,
[`protocolo/daq_protocolo.h`](protocolo/daq_protocolo.h), que comparten el
firmware y el software de la PC. Sus pruebas (`protocolo/prueba_protocolo.c`)
se ejecutan sin hardware e incluyen un modelo del esclavo que reproduce el
retardo de una trama en la respuesta.

### Firmware `dsPicDaq`

SPI1 opera como esclavo en modo de buffer estándar. Dos canales DMA mueven los
datos entre SPI1 y la RAM durante la transferencia: DMA0 recibe, disparado por
el evento de buffer de recepción lleno (`CHSEL = 0x02`), y DMA1 transmite,
disparado por el evento de buffer de transmisión vacío (`CHSEL = 0x03`). La
FIFO del modo mejorado no se utiliza porque en este dispositivo tiene cuatro
niveles en 8 bits, insuficientes para una trama completa.

`CS` (RB10) se asigna mediante PPS tanto a `SS1` como a `INT1`. La interrupción
del flanco de subida valida la trama, reinicia SPI1 (lo que vacía los buffers y
borra `SPIROV`), muestrea el QEI, construye la respuesta siguiente y vuelve a
programar el DMA. El lazo principal sólo escribe al DAC8554 cuando el valor
cambia. La terminal RD10 (LED0) permanece en alto durante la interrupción, lo
que permite medir su duración con un analizador lógico. Timer1 actúa como temporizador de vigilancia: si transcurren 50 ms sin
una trama válida, el DAC se lleva a 0 V y se reporta en el byte `status`.

El proyecto se compila con optimización `-O1` para acortar la interrupción de
fin de trama, que determina el intervalo mínimo entre tramas.

Antes de integrarlo con Simulink deben verificarse en la tarjeta, con el
programa de prueba y un analizador lógico:

- que el canal de transmisión entregue los ocho bytes sin repeticiones ni
  omisiones, en particular el segundo byte, cuyo disparo depende de si el
  evento del SPI se detecta por flanco o por nivel;
- la duración de la interrupción de fin de trama, medida entre el flanco de
  subida de `CS` y el término de la rutina;
- la ausencia de errores en al menos 100 000 tramas consecutivas.

### Programa de prueba

`herramientas/prueba_enlace` envía tramas sin Simulink y contabiliza los
errores de encabezado, CRC y número de secuencia, así como los que reporta el
dsPIC. Mide también la duración de cada transferencia. Por omisión la salida
analógica permanece en 0 V; la opción `-salida` genera una rampa de ±1 V para
verificarla con osciloscopio. Las instrucciones de compilación se encuentran
en el encabezado del archivo.

### Bloque de Simulink

`SIMULINK/DaqPic` contiene la S-Function `daqPic`, que sustituye a
`initializePic`, `leerPic` y `escribirPic`. El bloque abre el FT2232H, ejecuta
una transferencia por paso y, al terminar la simulación, deja la salida en
0 V. Al iniciar verifica que el dsPIC responda con el protocolo nuevo.

| Puerto | Señal |
|---|---|
| Entrada | Voltaje de salida [V], saturado a ±2.5 V |
| Salida 1 | Posición del encoder [cuentas], relativa al inicio de la simulación |
| Salida 2 | Errores de comunicación acumulados |
| Salida 3 | Pasos atrasados respecto al tiempo real |

Sus parámetros son el periodo de muestreo y la sincronización con el reloj de
la PC. La transferencia se realiza en la actualización del bloque, con el
voltaje del paso actual, y la posición recibida se entrega en el paso
siguiente. Así el bloque no tiene transmisión directa y puede emplearse en
lazo cerrado sin generar lazos algebraicos; el retardo total del lazo es de
dos periodos. `compilar.m` genera el archivo MEX y `crear_modelo.m` construye
un modelo de prueba.

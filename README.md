# DAQ-SPI

Emulación de una tarjeta de adquisición de datos tipo **Quanser** con un
**dsPIC33CK64MC105** (Curiosity Nano), controlada en tiempo real desde
**Simulink** a través de un puente USB–SPI **FT2232H**.


## Arquitectura

![Arquitectura del sistema](docs/arquitectura.png)

Fuente en TikZ: [`docs/arquitectura.tex`](docs/arquitectura.tex) (PDF: [`docs/arquitectura.pdf`](docs/arquitectura.pdf)).

- **PC:** Simulink llama a S-Functions en C/C++ que usan la biblioteca
  `libMPSSE-SPI` de FTDI para hablar con el FT2232H.
- **FT2232H:** maestro SPI a 1 MHz, modo 0, `CS` en `DBUS3`, activo en bajo,
  latency timer de 1 ms.
- **dsPIC:**
  - **SPI1 (esclavo):** recibe del FT2232H el valor para el DAC y le regresa
    la posición del encoder.
  - **SPI2 (maestro):** escribe el valor recibido al DAC de 16 bits.
  - **QEI1:** cuenta los pulsos del encoder del motor.

## Estructura del repositorio

```
DAQ-SPI/
├── MPLAB/                      Firmware del dsPIC (proyectos MPLAB X + MCC)
│   ├── dsPicReadEncoder.X/     Solo lectura del encoder → SPI1
│   ├── dsPicWriteEncoder.X/    Solo escritura SPI1 → DAC por SPI2
│   └── dsPicController.X/      Ambos combinados en un solo firmware
├── SIMULINK/                   Modelos y S-Functions para Windows
│   ├── ReadPic/                Lectura del encoder (versión original)
│   ├── WritePic/               Escritura al DAC (versión original)
│   ├── ReadPicModificado/      leerPic: lectura con handle compartido
│   ├── WritePicModificado/     initializePic + escribirPic: handle compartido
│   └── MergedPic/              Lectura y escritura en una sola S-Function
├── DRIVER FTDI/                Driver CDM de FTDI para Windows (v2.12.36.4)
└── docs/                       Diagrama de arquitectura (TikZ, PDF, PNG)
```

### S-Functions con handle compartido (`*Modificado/`)

Las versiones modificadas separan la apertura del FT2232H de las transferencias:

| S-Function | Función |
|---|---|
| `initializePic` | Abre el canal SPI del FT2232H y saca como señal un apuntador (`uint64`) a la estructura `HandleFTDI` |
| `leerPic` | Recibe el handle, lee 12 bytes por SPI y busca la trama del encoder (`0xAA 0x55` + 4 bytes) |
| `escribirPic` | Recibe el handle y un voltaje en ±2.5 V, y lo envía como trama `0xAA` + 16 bits |

`HandleFTDI` (en `ftdi_compartido.h`) incluye un mutex de Windows para que las
lecturas y escrituras no usen el FT2232H al mismo tiempo.

## Conexiones

| Señal | dsPIC | FT2232H | DAC |
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

Reloj del dsPIC: FRC interno de 8 MHz sin PLL (Fosc = 8 MHz, Fcy = 4 MHz).

## Protocolo SPI actual

**Escritura (PC → dsPIC):** 3 bytes por transferencia.

| Byte | Contenido |
|---|---|
| 0 | `0xAA` (encabezado) |
| 1 | Valor DAC, parte alta |
| 2 | Valor DAC, parte baja |

El voltaje se convierte como `valor = 13107 · (V + 2.5)`, es decir
−2.5 V → 0, 0 V → 32767 y +2.5 V → 65535. El dsPIC lo reenvía al DAC con
el byte de comando `0x10` seguido de los 16 bits.

**Lectura (dsPIC → PC):** la PC lee 12 bytes y busca la secuencia:

| Byte | Contenido |
|---|---|
| 0–1 | `0xAA 0x55` (encabezado) |
| 2–5 | Posición del QEI, `int32` little-endian |

## Compilación

### Firmware

Abrir el proyecto `.X` en **MPLAB X** con el compilador **XC-DSC** (o XC16) y
el pack `dsPIC33CK-MC_DFP`, y programar la Curiosity Nano con su depurador
integrado. Otra opción es copiar el `.hex` generado al disco USB `CURIOSITY`
que aparece al conectar la tarjeta.

### S-Functions

Los `.mexw64` ya compilados vienen incluidos para Windows de 64 bits. Para
recompilar, desde MATLAB en la carpeta del modelo (ejemplo; ajustar rutas de `libmpsse`):

```matlab
mex escribirPic.cpp libmpsse.lib
```

`libmpsse.dll` y `msvcr120.dll` deben estar en la misma carpeta que el modelo.
En Windows hay que instalar el driver de `DRIVER FTDI/`.

## Problemas conocidos

Cuando lectura y escritura se usan al mismo tiempo (`dsPicController` +
`leerPic`/`escribirPic`), la comunicación se corrompe. Causas identificadas en
el código:

1. **SPI es full-duplex, pero se usa como half-duplex.** La PC hace un
   `SPI_Write` y un `SPI_Read` por separado. Durante el `SPI_Write`, el dsPIC
   también saca bytes por MISO y gasta datos de encoder que la PC nunca lee.
   Durante el `SPI_Read`, la PC manda bytes de relleno que el dsPIC recibe
   como si fueran datos.
2. **El esclavo llena su FIFO de transmisión sin control.** `lecturaEncoder()`
   escribe 6 bytes cada vez que hay espacio, sin saber cuándo leerá la PC. La
   PC recibe valores viejos y desalineados, y por eso tiene que buscar el
   encabezado dentro de 12 bytes.
3. **Escrituras bloqueantes en el lazo principal.** `SPI1_ByteWrite()` se queda
   esperando mientras la FIFO esté llena, y eso solo cambia cuando la PC manda
   reloj. Mientras tanto, `escrituraDac()` no atiende la recepción.
4. **Desbordamiento de recepción.** Si la FIFO de recepción se llena (por el
   punto 3 o por los `printf`), se activa `SPIROV`. El driver de MCC nunca lo
   limpia, así que el dsPIC deja de recibir.
5. **No hay delimitación de tramas por `CS`.** El dsPIC no usa el flanco de
   `SS1` para saber dónde empieza una trama. Solo busca el byte `0xAA`, que
   también puede aparecer dentro de los datos.

El mutex del lado de la PC evita que dos S-Functions usen el FT2232H al mismo
tiempo, pero no resuelve los puntos anteriores, que ocurren en el dsPIC.

**Solución propuesta:** una sola transferencia full-duplex de longitud fija por
paso de Simulink (`SPI_ReadWrite`). En el dsPIC, SPI1 con DMA y buffers dobles
que se intercambian en el flanco de subida de `CS`, más un número de secuencia
y un CRC en cada trama.

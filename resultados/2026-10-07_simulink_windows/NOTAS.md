# Simulink en Windows con la tarjeta (2026-10-07)

PC con Windows 10 Pro 22H2, MATLAB R2025b Update 1 y Microsoft Visual C++ 2022
(Build Tools). Firmware dsPicDaq. Los bloques se compilaron con `compilar.m` y
los modelos se ejecutaron con `matlab -batch` por SSH (sin pantalla), 10 s a
1 kHz.

## Correcciones que salieron de esta prueba

- **Canal del FT2232H.** En esta PC libMPSSE enumeró la interfaz B como canal
  0 (`FT88FI80B`, LocId 7730) y la A como canal 1; el dsPIC sólo respondía
  0xFF. `protocolo/daq_canal.h` elige el canal cuyo número de serie termina
  en `A`. Con el canal 1 el dsPIC respondió con CRC válido y el eco de la
  secuencia.
- **mdlTerminate sin PWork.** Al evaluar los parámetros del bloque
  (`set_param` en `crear_modelo.m`), Simulink llamó a `mdlTerminate` de
  `daqPic` sin `mdlStart` y con PWork nulo: violación de acceso y MATLAB se
  cerró. Lo mismo aplicaba a `daqPicInicio` e `initializePic`.

## Resultados

| Modelo | Errores | Atrasos |
|---|---|---|
| `MergedPic.slx` del asesor, sin cambios (initializePic/escribirPic/leerPic, 1 V) | 0 | 1 |
| `DaqPicPrueba` con `load_system` | 0 | 2 a 5 |
| `DaqPicPrueba` con `open_system` | 4 a 10 | 44 a 45 |
| `DaqPicBloquesPrueba` con `open_system` | 4 | 41 |

Con el modelo abierto en el editor (sin pantalla, por `-batch`) Simulink se
detiene cada 0.25 s aproximadamente; cuando la pausa pasa de 50 ms expira la
vigilancia del dsPIC y la trama siguiente cuenta como error. Falta comprobar
si ocurre igual en la interfaz gráfica.

La posición permaneció en 0 en todas las corridas, como en Linux: el motor no
se movió (ver las notas de `2026-10-06_salida` sobre el cableado del DAC).

## Archivos

- `MergedPic_resultado.mat` (`t`, `y`, `voltaje`), `MergedPic_posicion.png`.
- `DaqPicPrueba_resultado.mat`, `DaqPicBloquesPrueba_resultado.mat`: una
  matriz `[t, valor]` por señal registrada (`daqPic_1` posición, `_2`
  errores, `_3` atrasos; `daqPicLeer_*` igual).
- `*_diagrama.png`: los modelos exportados con `print -s`.

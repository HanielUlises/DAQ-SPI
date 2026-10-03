% Crea el modelo de prueba DaqPicPrueba.slx.
%
% Una onda senoidal de 0.5 V y 0.5 Hz se aplica al motor durante 10 s. Se
% grafican el voltaje y la posición del encoder, y se muestran los contadores
% de errores de comunicación y de pasos atrasados, que deben permanecer en 0.

carpeta = fileparts(mfilename('fullpath'));
anterior = cd(carpeta);
regresar = onCleanup(@() cd(anterior));

if exist(['daqPic.' mexext], 'file') ~= 3
    compilar;
end

modelo = 'DaqPicPrueba';
if bdIsLoaded(modelo)
    close_system(modelo, 0);
end
new_system(modelo);
set_param(modelo, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.001', 'StopTime', '10');

add_block('simulink/Sources/Sine Wave', [modelo '/Voltaje'], ...
    'Amplitude', '0.5', 'Frequency', '2*pi*0.5', 'SampleTime', '0.001', ...
    'Position', [40 95 90 125]);

% Parámetros: periodo de muestreo [s], sincronizar con tiempo real (0/1)
add_block('simulink/User-Defined Functions/S-Function', [modelo '/daqPic'], ...
    'Parameters', '0.001, 1', 'FunctionName', 'daqPic', ...
    'Position', [180 85 280 135]);

add_block('simulink/Sinks/Scope', [modelo '/Voltaje y posicion'], ...
    'NumInputPorts', '2', 'Position', [400 40 440 100]);
add_block('simulink/Sinks/Display', [modelo '/Errores'], ...
    'Position', [400 125 480 145]);
add_block('simulink/Sinks/Display', [modelo '/Atrasos'], ...
    'Position', [400 165 480 185]);

add_line(modelo, 'Voltaje/1', 'daqPic/1', 'autorouting', 'on');
add_line(modelo, 'Voltaje/1', 'Voltaje y posicion/1', 'autorouting', 'on');
add_line(modelo, 'daqPic/1', 'Voltaje y posicion/2', 'autorouting', 'on');
add_line(modelo, 'daqPic/2', 'Errores/1', 'autorouting', 'on');
add_line(modelo, 'daqPic/3', 'Atrasos/1', 'autorouting', 'on');

save_system(modelo, fullfile(carpeta, [modelo '.slx']));
open_system(modelo);
fprintf('Listo: %s\n', fullfile(carpeta, [modelo '.slx']));

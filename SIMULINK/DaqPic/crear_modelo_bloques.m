% Crea el modelo de prueba DaqPicBloquesPrueba.slx, equivalente a
% DaqPicPrueba.slx pero con la versión en tres bloques: daqPicInicio abre el
% FT2232H y ejecuta la transferencia de cada paso, daqPicEscribir le entrega
% el voltaje y daqPicLeer la posición, los errores y los pasos atrasados.
%
% Una onda senoidal de 0.5 V y 0.5 Hz se aplica al motor durante 10 s. Los
% contadores de errores de comunicación y de pasos atrasados deben
% permanecer en 0.

carpeta = fileparts(mfilename('fullpath'));
anterior = cd(carpeta);
regresar = onCleanup(@() cd(anterior));

bloques = {'daqPicInicio', 'daqPicEscribir', 'daqPicLeer'};
if any(cellfun(@(b) exist([b '.' mexext], 'file') ~= 3, bloques))
    compilar;
end

modelo = 'DaqPicBloquesPrueba';
if bdIsLoaded(modelo)
    close_system(modelo, 0);
end
new_system(modelo);
set_param(modelo, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.001', 'StopTime', '10');

% Parámetros: periodo de muestreo [s], sincronizar con tiempo real (0/1)
add_block('simulink/User-Defined Functions/S-Function', [modelo '/daqPicInicio'], ...
    'Parameters', '0.001, 1', 'FunctionName', 'daqPicInicio', ...
    'Position', [40 200 140 240]);

add_block('simulink/Sources/Sine Wave', [modelo '/Voltaje'], ...
    'Amplitude', '0.5', 'Frequency', '2*pi*0.5', 'SampleTime', '0.001', ...
    'Position', [40 85 90 115]);

add_block('simulink/User-Defined Functions/S-Function', [modelo '/daqPicEscribir'], ...
    'FunctionName', 'daqPicEscribir', 'Position', [240 80 340 130]);
add_block('simulink/User-Defined Functions/S-Function', [modelo '/daqPicLeer'], ...
    'FunctionName', 'daqPicLeer', 'Position', [240 195 340 245]);

add_block('simulink/Sinks/Scope', [modelo '/Voltaje y posicion'], ...
    'NumInputPorts', '2', 'Position', [460 40 500 100]);
add_block('simulink/Sinks/Display', [modelo '/Errores'], ...
    'Position', [460 210 540 230]);
add_block('simulink/Sinks/Display', [modelo '/Atrasos'], ...
    'Position', [460 250 540 270]);

add_line(modelo, 'daqPicInicio/1', 'daqPicEscribir/1', 'autorouting', 'on');
add_line(modelo, 'daqPicInicio/1', 'daqPicLeer/1', 'autorouting', 'on');
add_line(modelo, 'Voltaje/1', 'daqPicEscribir/2', 'autorouting', 'on');
add_line(modelo, 'Voltaje/1', 'Voltaje y posicion/1', 'autorouting', 'on');
add_line(modelo, 'daqPicLeer/1', 'Voltaje y posicion/2', 'autorouting', 'on');
add_line(modelo, 'daqPicLeer/2', 'Errores/1', 'autorouting', 'on');
add_line(modelo, 'daqPicLeer/3', 'Atrasos/1', 'autorouting', 'on');

save_system(modelo, fullfile(carpeta, [modelo '.slx']));
open_system(modelo);
fprintf('Listo: %s\n', fullfile(carpeta, [modelo '.slx']));

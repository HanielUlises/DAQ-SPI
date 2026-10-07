function datos = crear_modelo_pid(varargin)
% Crea DaqPicPid.slx: lazo cerrado de posición con PID, equivalente al modelo
% de lazo cerrado de herramientas/visor_enlace (control.h), para comparar
% Simulink con el visor sobre la misma tarjeta:
%
%   referencia -> (+) -> Discrete PID Controller -> daqPic -> 1/escala -> y
%                 (-) <--------------------------------------------'
%
% El PID está en forma paralela, con integrador de Euler hacia adelante,
% filtro de la derivada de Euler hacia atrás con coeficiente N, salida
% limitada a [u_min, u_max] y anti-windup por sujeción, como en el visor. La
% referencia se calcula aquí con la misma fórmula que el generador del visor
% y entra muestra por muestra (Repeating Sequence Stair), sin interpolar.
% Las ganancias están en V por unidad de posición; la posición de daqPic,
% relativa al inicio de la simulación, se divide entre las cuentas por
% unidad.
%
%   crear_modelo_pid                        construye el modelo y lo abre
%   datos = crear_modelo_pid('correr', true, 'archivo', 'pid.mat', ...)
%
% Con 'correr' simula el modelo y devuelve t, u, y, r, e, errores, atrasos y
% escala; con 'archivo' los guarda en formato MAT 4 (save -v4), que
% visor_enlace abre para superponerlos a su propia corrida, y la figura en
% un PNG con el mismo nombre.
%
% Parámetros (nombre, valor) y sus valores por omisión, los del visor:
%   Ts 0.001, tf 10, tiempo_real 1
%   unidad 'cuentas' ('vueltas', 'grados', 'rad'), cuentas_por_vuelta 4096
%   forma 'cuadrada' ('nada', 'escalon', 'rampa', 'seno'), amplitud 500,
%   frecuencia 0.5, desplazamiento 0, retardo 0
%   P 0.003, I 0, D 0, N 100, d_medicion false
%   u_min -2.5, u_max 2.5
%   correr false, archivo ''
%
% La corrida equivalente del visor, con los valores por omisión:
%   visor_enlace -periodo 1000 -tf 10 -cerrado -kp 0.003 -salida -iniciar
% Con la derivada (D > 0), la referencia debe empezar en su desplazamiento
% (retardo > 0 o forma 'escalon' con retardo): el visor arranca la derivada
% en cero y el bloque de Simulink con su estado en cero.

p = struct('Ts', 0.001, 'tf', 10, 'tiempo_real', 1, ...
    'unidad', 'cuentas', 'cuentas_por_vuelta', 4096, ...
    'forma', 'cuadrada', 'amplitud', 500, 'frecuencia', 0.5, ...
    'desplazamiento', 0, 'retardo', 0, ...
    'P', 0.003, 'I', 0, 'D', 0, 'N', 100, 'd_medicion', false, ...
    'u_min', -2.5, 'u_max', 2.5, 'correr', false, 'archivo', '');
if mod(numel(varargin), 2) ~= 0
    error('crear_modelo_pid:argumentos', 'Los parámetros van en pares nombre, valor.');
end
for k = 1:2:numel(varargin)
    nombre = varargin{k};
    if ~isfield(p, nombre)
        error('crear_modelo_pid:argumentos', 'Parámetro desconocido: %s', nombre);
    end
    p.(nombre) = varargin{k + 1};
end

carpeta = fileparts(mfilename('fullpath'));
anterior = cd(carpeta);
regresar = onCleanup(@() cd(anterior));

if exist(['daqPic.' mexext], 'file') ~= 3
    compilar;
end

escala = cuentas_por_unidad(p);
n = floor(p.tf / p.Ts + 1e-9) + 1;
t = (0:n - 1)' * p.Ts;
referencia = generador(p, t);

modelo = 'DaqPicPid';
if bdIsLoaded(modelo)
    close_system(modelo, 0);
end
new_system(modelo);
set_param(modelo, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num(p.Ts), 'StopTime', num(p.tf), ...
    'SaveTime', 'on', 'ReturnWorkspaceOutputs', 'on');
espacio = get_param(modelo, 'ModelWorkspace');
assignin(espacio, 'referencia', referencia');

add_block('simulink/Sources/Repeating Sequence Stair', [modelo '/Referencia'], ...
    'OutValues', 'referencia', 'tsamp', num(p.Ts), 'Position', [30 95 80 125]);

comunes = {'SampleTime', num(p.Ts), 'Form', 'Parallel', ...
    'P', num(p.P), 'I', num(p.I), 'D', num(p.D), 'N', num(p.N), ...
    'IntegratorMethod', 'Forward Euler', 'FilterMethod', 'Backward Euler', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', num(p.u_max), ...
    'LowerSaturationLimit', num(p.u_min), 'AntiWindupMode', 'clamping'};
if p.d_medicion
    % 2DOF con c = 0: la derivada actúa sólo sobre -y
    add_block('simulink/Discrete/Discrete PID Controller (2DOF)', [modelo '/PID'], ...
        comunes{:}, 'b', '1', 'c', '0', 'Position', [170 90 250 145]);
    entrada_pid = {'Referencia/1', 'PID/1'};
    retro = 'PID/2';
else
    add_block('simulink/Math Operations/Sum', [modelo '/Suma'], ...
        'IconShape', 'round', 'Inputs', '|+-', 'Position', [120 100 140 120]);
    add_block('simulink/Discrete/Discrete PID Controller', [modelo '/PID'], ...
        comunes{:}, 'Position', [180 90 260 130]);
    entrada_pid = {'Referencia/1', 'Suma/1'; 'Suma/1', 'PID/1'};
    retro = 'Suma/2';
end

% Parámetros de daqPic: periodo de muestreo [s], sincronizar con tiempo real (0/1)
add_block('simulink/User-Defined Functions/S-Function', [modelo '/daqPic'], ...
    'Parameters', sprintf('%s, %d', num(p.Ts), p.tiempo_real ~= 0), ...
    'FunctionName', 'daqPic', 'Position', [320 85 420 135]);
add_block('simulink/Math Operations/Gain', [modelo '/Cuentas a unidad'], ...
    'Gain', ['1/' num(escala)], 'Position', [470 95 520 125]);

add_block('simulink/Sinks/Scope', [modelo '/Scope'], ...
    'NumInputPorts', '3', 'Position', [620 20 660 90]);
try
    set_param([modelo '/Scope'], 'LayoutDimensions', '[3 1]');
catch
    % Versiones sin esta propiedad: un solo eje con las tres señales
end
add_block('simulink/Sinks/Display', [modelo '/Errores'], 'Position', [620 160 700 180]);
add_block('simulink/Sinks/Display', [modelo '/Atrasos'], 'Position', [620 200 700 220]);

senales = {'r', 'Referencia/1', [560 260 610 280]; 'u', 'PID/1', [560 300 610 320]; ...
           'y', 'Cuentas a unidad/1', [560 340 610 360]; ...
           'errores', 'daqPic/2', [740 160 790 180]; 'atrasos', 'daqPic/3', [740 200 790 220]};
for k = 1:size(senales, 1)
    add_block('simulink/Sinks/To Workspace', [modelo '/' senales{k, 1}], ...
        'VariableName', senales{k, 1}, 'SaveFormat', 'Array', 'MaxDataPoints', 'inf', ...
        'Position', senales{k, 3});
end

for k = 1:size(entrada_pid, 1)
    add_line(modelo, entrada_pid{k, 1}, entrada_pid{k, 2}, 'autorouting', 'on');
end
add_line(modelo, 'PID/1', 'daqPic/1', 'autorouting', 'on');
add_line(modelo, 'daqPic/1', 'Cuentas a unidad/1', 'autorouting', 'on');
add_line(modelo, 'Cuentas a unidad/1', retro, 'autorouting', 'on');
add_line(modelo, 'Referencia/1', 'Scope/1', 'autorouting', 'on');
add_line(modelo, 'PID/1', 'Scope/2', 'autorouting', 'on');
add_line(modelo, 'Cuentas a unidad/1', 'Scope/3', 'autorouting', 'on');
add_line(modelo, 'daqPic/2', 'Errores/1', 'autorouting', 'on');
add_line(modelo, 'daqPic/3', 'Atrasos/1', 'autorouting', 'on');
for k = 1:size(senales, 1)
    add_line(modelo, senales{k, 2}, [senales{k, 1} '/1'], 'autorouting', 'on');
end

save_system(modelo, fullfile(carpeta, [modelo '.slx']));
fprintf('Listo: %s (referencia %s, P %g, I %g, D %g, N %g, unidad %s)\n', ...
    fullfile(carpeta, [modelo '.slx']), p.forma, p.P, p.I, p.D, p.N, p.unidad);

datos = struct();
if ~p.correr
    open_system(modelo);
    return;
end

salida = sim(modelo);
datos.t = columna(salida.tout);
datos.u = columna(salida.get('u'));
datos.y = columna(salida.get('y'));
datos.r = columna(salida.get('r'));
datos.e = datos.r - datos.y;
datos.errores = columna(salida.get('errores'));
datos.atrasos = columna(salida.get('atrasos'));
datos.escala = escala;

ultimo = max(1, numel(datos.y));
fprintf(['Muestras: %d en %g s. Errores: %g. Atrasos: %g.\n' ...
         'y final %.4g %s (r = %.4g), máximo %.4g\n'], numel(datos.t), datos.t(end), ...
        datos.errores(end), datos.atrasos(end), datos.y(ultimo), p.unidad, datos.r(ultimo), ...
        max(datos.y));
if datos.errores(end) == 0 && datos.atrasos(end) == 0
    fprintf('Resultado: SIN ERRORES\n');
else
    fprintf('Resultado: CON ERRORES O ATRASOS\n');
end

if ~isempty(p.archivo)
    t = datos.t; u = datos.u; y = datos.y; r = datos.r; e = datos.e; %#ok<NASGU>
    errores = datos.errores; atrasos = datos.atrasos; escala = datos.escala; %#ok<NASGU>
    save(p.archivo, 't', 'u', 'y', 'r', 'e', 'errores', 'atrasos', 'escala', '-v4');
    figura = figure('Visible', 'off', 'Position', [100 100 900 700]);
    subplot(3, 1, 1);
    plot(datos.t, datos.u);
    grid on; ylabel('u [V]'); title('Voltaje');
    subplot(3, 1, 2);
    plot(datos.t, datos.r, datos.t, datos.y);
    grid on; ylabel(['[' p.unidad ']']); legend('r', 'y'); title('Posición');
    subplot(3, 1, 3);
    plot(datos.t, datos.e);
    grid on; ylabel(['[' p.unidad ']']); xlabel('t [s]'); title('Error e = r - y');
    [carpeta_archivo, base] = fileparts(p.archivo);
    exportgraphics(figura, fullfile(carpeta_archivo, [base '.png']));
    close(figura);
    fprintf('Guardado: %s\n', p.archivo);
end
end

function s = num(x)
s = sprintf('%.17g', x);
end

function c = columna(x)
c = double(x(:));
end

function k = cuentas_por_unidad(p)
switch p.unidad
    case 'cuentas', k = 1;
    case 'vueltas', k = p.cuentas_por_vuelta;
    case 'grados', k = p.cuentas_por_vuelta / 360;
    case 'rad', k = p.cuentas_por_vuelta / (2 * pi);
    otherwise
        error('crear_modelo_pid:unidad', 'Unidad desconocida: %s', p.unidad);
end
end

% Misma fórmula que Generador::valor de herramientas/visor_enlace/control.h
function v = generador(p, t)
v = zeros(size(t));
if strcmp(p.forma, 'nada')
    return;
end
v(:) = p.desplazamiento;
tt = t - p.retardo;
activo = tt >= 0;
if p.frecuencia > 0
    fase = tt * p.frecuencia - floor(tt * p.frecuencia);
else
    fase = zeros(size(t));
end
a = p.amplitud;
switch p.forma
    case 'escalon'
        w = a * ones(size(t));
    case 'rampa'
        w = a * ((fase < 0.5) .* (-1 + 4 * fase) + (fase >= 0.5) .* (3 - 4 * fase));
    case 'seno'
        w = a * sin(2 * pi * fase);
    case 'cuadrada'
        w = a * (2 * (fase < 0.5) - 1);
    otherwise
        error('crear_modelo_pid:forma', 'Forma desconocida: %s', p.forma);
end
v(activo) = w(activo) + p.desplazamiento;
end

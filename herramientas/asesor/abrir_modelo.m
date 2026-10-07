function abrir_modelo(modelo)
% Abre el modelo de esta carpeta con ella como carpeta actual de MATLAB, para
% que Simulink use los initializePic, escribirPic y leerPic de aquí y no los
% de otra carpeta.
%
% Antes de abrirlo verifica que cada bloque esté compilado en esta carpeta;
% si falta o el .mexw64 no carga con esta versión de MATLAB, lo recompila
% con compilar_bloques.

carpeta = fileparts(mfilename('fullpath'));
cd(carpeta);
clear mex
rehash

bloques = {'initializePic', 'escribirPic', 'leerPic'};
for k = 1:numel(bloques)
    b = bloques{k};
    if ~isfile(fullfile(carpeta, [b '.cpp']))
        continue;
    end
    ruta = which(b);
    if exist(b, 'file') ~= 3 || ~strcmpi(fileparts(ruta), carpeta)
        fprintf('%s no está compilado en esta carpeta (MATLAB encontraba: %s).\n', b, ruta);
        compilar_bloques;
        break;
    end
end

open_system(fullfile(carpeta, [modelo '.slx']));
try
    set_param(modelo, 'SimulationCommand', 'update');
catch err
    fprintf('Los bloques no cargaron con este MATLAB:\n  %s\nSe recompilan.\n', err.message);
    close_system(modelo, 0);
    compilar_bloques;
    open_system(fullfile(carpeta, [modelo '.slx']));
    set_param(modelo, 'SimulationCommand', 'update');
end

for k = 1:numel(bloques)
    if isfile(fullfile(carpeta, [bloques{k} '.cpp']))
        fprintf('%s: %s\n', bloques{k}, which(bloques{k}));
    end
end
fprintf('Listo: %s abierto con %s como carpeta actual. Run para simular.\n', modelo, carpeta);
end

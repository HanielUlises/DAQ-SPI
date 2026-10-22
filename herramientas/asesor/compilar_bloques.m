function compilar_bloques
% Compila en esta carpeta los bloques initializePic, escribirPic y leerPic
% cuyos .cpp estén aquí. Necesitan en la misma carpeta ftdi_compartido.h,
% daq_protocolo.h, daq_canal.h, ftd2xx.h, libmpsse_spi.h y libmpsse.lib.
% Los tres usan libMPSSE y registran el tipo de la llave con la biblioteca
% de punto fijo de Simulink (-lfixedpoint).
%
% Requiere un compilador de C++ configurado para MEX (mex -setup C++).

carpeta = fileparts(mfilename('fullpath'));
anterior = cd(carpeta);
regresar = onCleanup(@() cd(anterior));

% Libera los .mexw64 cargados para poder reemplazarlos
clear mex

bloques = {'initializePic', 'escribirPic', 'leerPic'};
for k = 1:numel(bloques)
    b = bloques{k};
    if ~isfile(fullfile(carpeta, [b '.cpp']))
        continue;
    end
    fprintf('Compilando %s...\n', b);
    mex('-silent', [b '.cpp'], 'libmpsse.lib', '-lfixedpoint');
end
fprintf('Listo: bloques compilados en %s\n', carpeta);
end

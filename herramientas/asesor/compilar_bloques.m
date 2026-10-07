function compilar_bloques
% Compila en esta carpeta los bloques initializePic, escribirPic y leerPic
% cuyos .cpp estén aquí. Los .cpp son autónomos: sólo necesitan ftd2xx.h,
% libmpsse_spi.h y libmpsse.lib en la misma carpeta.
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
    if strcmp(b, 'initializePic')
        mex('-silent', [b '.cpp'], 'libmpsse.lib');
    else
        mex('-silent', [b '.cpp']);
    end
end
fprintf('Listo: bloques compilados en %s\n', carpeta);
end

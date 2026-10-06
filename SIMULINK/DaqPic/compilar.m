% Compila las S-Functions de la DAQ: daqPic (un solo bloque) y la versión en
% tres bloques, daqPicInicio, daqPicEscribir y daqPicLeer.
%
% Requiere un compilador de C++ configurado para MEX (mex -setup C++). Si un
% modelo está abierto y ya se simuló, ejecutar antes "clear mex" para liberar
% los archivos .mexw64.

carpeta = fileparts(mfilename('fullpath'));
anterior = cd(carpeta);
regresar = onCleanup(@() cd(anterior));

mex('daqPic.cpp', 'libmpsse.lib');
mex('daqPicInicio.cpp', 'libmpsse.lib');
mex('daqPicEscribir.cpp');
mex('daqPicLeer.cpp');

fprintf('Listo: daqPic, daqPicInicio, daqPicEscribir y daqPicLeer en %s\n', carpeta);

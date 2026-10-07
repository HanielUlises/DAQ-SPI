% Compila las S-Functions de la DAQ: daqPic (un solo bloque), la versión en
% tres bloques, daqPicInicio, daqPicEscribir y daqPicLeer, y los mismos tres
% bloques con los nombres y puertos de los anteriores: initializePic,
% escribirPic y leerPic.
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
mex('initializePic.cpp', 'libmpsse.lib');
mex('escribirPic.cpp');
mex('leerPic.cpp');

fprintf(['Listo: daqPic, daqPicInicio, daqPicEscribir, daqPicLeer, initializePic, ' ...
         'escribirPic y leerPic en %s\n'], carpeta);

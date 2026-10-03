% Compila la S-Function daqPic.
%
% Requiere un compilador de C++ configurado para MEX (mex -setup C++). Si el
% modelo está abierto y ya se simuló, ejecutar antes "clear mex" para liberar
% daqPic.mexw64.

carpeta = fileparts(mfilename('fullpath'));
anterior = cd(carpeta);
regresar = onCleanup(@() cd(anterior));

mex('daqPic.cpp', 'libmpsse.lib');

fprintf('Listo: %s\n', fullfile(carpeta, ['daqPic.' mexext]));

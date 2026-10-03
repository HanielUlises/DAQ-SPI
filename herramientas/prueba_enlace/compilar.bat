@echo off
rem Compila prueba_enlace.exe con Visual Studio.
rem Ejecutar desde "x64 Native Tools Command Prompt for VS".
rem Ya se incluye un ejecutable compilado; esto solo es necesario si se modifica el codigo.
setlocal
cd /d "%~dp0"
cl /nologo /O2 /EHsc /utf-8 /W4 /I..\..\SIMULINK\MergedPic prueba_enlace.cpp ..\..\SIMULINK\MergedPic\libmpsse.lib
if errorlevel 1 exit /b 1
del /q prueba_enlace.obj 2>nul
copy /y ..\..\SIMULINK\MergedPic\libmpsse.dll . >nul
copy /y ..\..\SIMULINK\MergedPic\msvcr120.dll . >nul
echo Listo: prueba_enlace.exe

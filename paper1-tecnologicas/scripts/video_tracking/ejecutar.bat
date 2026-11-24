@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo ============================================================
echo  Seguimiento del marcador en los videos del Paper 1
echo ============================================================
set "CARPETA=C:\Universidad\Servo\Resultados Paper 1"
if not "%~1"=="" set "CARPETA=%~1"

set "PYBASE=python"
where py >nul 2>nul && set "PYBASE=py -3"
if exist ".venv" if not exist ".venv\Scripts\python.exe" rmdir /s /q ".venv"
if not exist ".venv\Scripts\python.exe" (
    echo [1/3] Creando entorno .venv con %PYBASE% ...
    %PYBASE% -m venv .venv || goto :error
)
set "PY=.venv\Scripts\python.exe"
echo [2/3] Instalando numpy y opencv ...
"%PY%" -m pip install --upgrade pip >nul
"%PY%" -m pip install numpy opencv-python || goto :error
echo [3/3] Procesando los videos de "%CARPETA%" ...
echo       Puede tardar varias horas. Si se interrumpe, vuelva a ejecutar: sigue con los que faltan.
"%PY%" seguir_todos.py "%CARPETA%" || goto :error
echo.
echo Terminado. Los CSV quedaron en "%CARPETA%\marcador".
pause
exit /b 0
:error
echo.
echo *** Hubo un error. Revise los mensajes de arriba. ***
pause
exit /b 1

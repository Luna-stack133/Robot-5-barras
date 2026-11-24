@echo off
REM ==========================================================================
REM  Crea el ejecutable de Windows de R5B Lab y un .zip listo para compartir.
REM  Uso: doble clic en este archivo (o ejecutarlo desde la terminal de VS Code).
REM  Resultado: dist\R5B_Lab_Windows.zip  (dentro: carpeta "R5B Lab" con "R5B Lab.exe")
REM ==========================================================================
setlocal
cd /d "%~dp0"

REM Python de Windows: se prefiere el lanzador "py" (python.org) porque el "python" del PATH
REM puede ser el de MSYS2/MinGW, que no trae pip y crea entornos sin carpeta Scripts.
set "PYBASE=python"
where py >nul 2>nul && set "PYBASE=py -3"

REM Un .venv incompleto (por ejemplo, creado con el Python de MSYS2) se borra y se vuelve a crear
if exist ".venv" if not exist ".venv\Scripts\python.exe" (
    echo Borrando un .venv incompleto ...
    rmdir /s /q ".venv"
)
if not exist ".venv\Scripts\python.exe" (
    echo [1/4] Creando entorno virtual .venv con: %PYBASE% ...
    %PYBASE% -m venv .venv || goto :error
)
set "PY=.venv\Scripts\python.exe"
if not exist "%PY%" (
    echo No se pudo crear .venv\Scripts\python.exe. Instale Python desde python.org.
    goto :error
)

echo [2/4] Instalando dependencias ...
"%PY%" -m pip install --upgrade pip >nul
"%PY%" -m pip install -r requirements.txt pyinstaller || goto :error

echo [3/4] Generando el ejecutable (tarda 1 a 3 minutos) ...
if exist build rmdir /s /q build
if exist "dist\R5B Lab" rmdir /s /q "dist\R5B Lab"
"%PY%" -m PyInstaller --noconfirm --clean --windowed ^
    --name "R5B Lab" ^
    --icon "r5b\assets\r5b.ico" ^
    --add-data "r5b\assets;r5b\assets" ^
    --hidden-import serial.tools.list_ports ^
    main.py || goto :error

echo [4/4] Comprimiendo para compartir ...
if exist "dist\R5B_Lab_Windows.zip" del "dist\R5B_Lab_Windows.zip"
powershell -NoProfile -Command "Compress-Archive -Path 'dist\R5B Lab' -DestinationPath 'dist\R5B_Lab_Windows.zip'" || goto :error

echo.
echo Listo. Archivo para compartir:  dist\R5B_Lab_Windows.zip
echo Para probarlo aqui mismo:       dist\R5B Lab\R5B Lab.exe
echo.
pause
exit /b 0

:error
echo.
echo *** Hubo un error. Revise los mensajes de arriba. ***
pause
exit /b 1

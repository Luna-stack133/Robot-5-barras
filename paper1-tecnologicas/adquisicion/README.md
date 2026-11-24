# Software de adquisición usado en el Paper 1

Las corridas de `data/raw/` se tomaron con estas versiones exactas (copias verificadas por suma MD5 contra las del PC del equipo):

| Carpeta | Qué es | Cómo se usó |
|---|---|---|
| `firmware_1.3/robot_5bar_esp32/` | Firmware de la ESP32-S3 (Arduino; FastAccelStepper). Genera STEP/DIR, lee la BNO055 a 100 Hz y envía la telemetría | Cargado en todas las sesiones S1–S4, con la corrección con IMU **apagada** (modo Apagada). El EKF y la corrección que trae son del Paper 2 y no se usaron aquí |
| `R5B_Lab_0.4.1/` | App de escritorio (Python, PySide) para mover el robot, lanzar las excitaciones/trayectorias y guardar cada corrida (`imu.csv`, `telemetria.csv`, `info.json`) | Todas las sesiones. Botón "Cargar excitación aquí" para P → P + Δ → P |

Instrucciones de compilación y uso: `firmware_1.3/LEEME_v1.3.md` y `R5B_Lab_0.4.1/README.md`.
La versión que evoluciona del firmware y la GUI vive en las carpetas `firmware/` y `gui/` de la raíz del repositorio.

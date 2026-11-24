# Firmware de la ESP32-S3

| Carpeta | Versión | Estado | Qué tiene |
|---|---|---|---|
| `robot_5bar_esp32/` | **1.3** | **En uso** (todas las sesiones desde el 15 nov, incluido el Paper 1) | Trayectoria continua y streaming de pulsos (FastAccelStepper), modos de ensamblaje, BNO055 a 100 Hz, EKF y corrección con IMU (arranca Apagada). Ver `LEEME_v1.3.md` |
| `v1.4_prealimentacion/robot_5bar_esp32/` | 1.4 | En desarrollo: validada en el PC, no probada en el robot | La 1.3 más la prealimentación por modelo (`CTL FF`, `FFGAN`, `FFMAX`, `MOD`). Ver `LEEME_v1.4.md` |

Compilar: Arduino IDE, placa *ESP32S3 Dev Module*, librería **FastAccelStepper** (gin66). La carpeta del sketch debe llamarse `robot_5bar_esp32` y tener todos los `.h` junto al `.ino`. No usar *Erase All Flash* (borra la calibración de la IMU).

La copia exacta usada en el Paper 1 también está en `paper1-tecnologicas/adquisicion/firmware_1.3/`.

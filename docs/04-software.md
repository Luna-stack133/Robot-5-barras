# 04 · Software

| Módulo | Ruta | Lenguaje | Corre en |
|---|---|---|---|
| Firmware del robot 1.3 (en uso) | `firmware/robot_5bar_esp32/` | C++ (Arduino) | ESP32-S3 |
| Núcleo cinemático | `firmware/robot_5bar_esp32/kinematics_5bar.h` | C++ (solo `<cmath>`) | ESP32-S3 (y pruebas en PC) |
| IMU, EKF y ley de corrección | `imu_bno055.h`, `ekf_5b.h`, `control_imu.h`, `lqr_tabla.h` | C++ | ESP32-S3 (núcleo 0) |
| Firmware 1.4 (prealimentación, en desarrollo) | `firmware/v1.4_prealimentacion/robot_5bar_esp32/` | C++ (Arduino) | ESP32-S3 |
| App de escritorio R5B Lab 0.4.1 | `gui/R5B_Lab/` | Python 3 + PySide6 | PC (Windows) |
| Etapa 0 de la IMU (BNO055 en reposo) | `imu/etapa0_bno055/` | C++ + Python | ESP32-S3 + PC |
| Etapa 1 (IMU frente a la cinemática, golpes) | `imu/etapa1/` | Python | PC |
| Etapa 2 (modelo, identificación, EKF offline, video) | `imu/etapa2_modelo_ekf/` | Python | PC |
| Análisis del Paper 1 | `paper1-tecnologicas/scripts/` | Python | PC |
| Simulación de trayectorias | `simulacion/matlab/` | MATLAB | PC |
| Configuración de los servos | JMC `JmcServoPcControl` 1.7.6 (no incluido) | — | PC (Windows, RS232) |

## Firmware del robot (`firmware/robot_5bar_esp32`, v1.3)

**Requisitos:** Arduino IDE con el core Arduino-ESP32, placa *ESP32S3 Dev Module*, librería **FastAccelStepper** (gin66) desde el Library Manager. Todos los `.h` deben estar en la misma carpeta que el `.ino`. Al conectar debe aparecer `OK ready - 5bar ESP32-S3 v1.3`. No usar *Erase All Flash*: borra la calibración guardada de la BNO055.

**Qué hace:**
- Recibe comandos de texto por USB (115200 bps).
- Valida los puntos (alcance, singularidad, límites articulares) y el modo de ensamblaje.
- Recorre las figuras como trazos continuos con perfil suave y frena solo en las esquinas; comanda los pulsos por streaming cada 4 ms con FastAccelStepper.
- Cambia de modo de ensamblaje (`MODE n`) con un movimiento articular verificado.
- Lee la BNO055 a 100 Hz en una tarea del núcleo 0 y envía las muestras (`I …`) con los ángulos de los motores del mismo instante.
- Corre el EKF y la ley de corrección con IMU (modos Apagada / Sombra / Activa; arranca Apagada).
- Envía telemetría `STATE` a 50 Hz (20 Hz mientras se envía la IMU).
- Fija el cero por utillaje (`SETHOME`) y permite mover un eje a mano (`JOG`).

**Parámetros que se ajustan en el código:**

| Constante | Archivo | Valor actual | Qué es |
|---|---|---|---|
| `L1, L2, L3, L4, D` | `kinematics_5bar.h` | 0,120 / 0,180 / 0,180 / 0,120 / 0,150 m | Geometría v3 |
| `STEPS_PER_REV` | `kinematics_5bar.h` | 8000 | Igual a la resolución configurada en el servo |
| `SIN_MIN` | `kinematics_5bar.h` | 0,15 | Margen de singularidad |
| `PIN_PUL1/DIR1/PUL2/DIR2` | `.ino` | 4 / 5 / 15 / 16 | Pines de los servos |
| `PIN_SDA / PIN_SCL / I2C_HZ` | `imu_bno055.h` | 8 / 9 / 100 kHz | Bus de la IMU |
| `DIR_SIGN1/2` | `.ino` | +1 / +1 | Invertir si un motor gira al revés |
| `TH1_HOME_DEG / TH2_HOME_DEG` | `.ino` | 90° / 90° | Ángulos que deja el utillaje |
| `TH1_MIN…TH2_MAX` | `.ino` | [−170°, 350°] | Límites articulares blandos |
| `g_Vmax / g_Amax` | `.ino` | 0,10 m/s / 0,20 m/s² | Velocidad y aceleración iniciales (`SETV`) |
| `CORNER_DEG` | `.ino` | 30° | Ángulo a partir del cual el robot frena en una esquina |

El protocolo de comandos está en [05-arquitectura-software.md](05-arquitectura-software.md#protocolo-serie). Detalles de la versión en `LEEME_v1.3.md`.

**Firmware 1.4** (`firmware/v1.4_prealimentacion/`): la 1.3 más la prealimentación por modelo (`CTL FF`, `CTL FFGAN`, `CTL FFMAX`, `CTL MOD`) y 6 campos más en la línea `I`. Con la prealimentación apagada los pulsos son idénticos a los de la 1.3. Validado en el PC, **todavía no probado en el robot**; trae los parámetros del modelo anterior al mantenimiento, que se actualizan con `CTL MOD`.

## App de escritorio R5B Lab (`gui/R5B_Lab`, v0.4.1)

Reemplaza a la GUI de Tkinter del inicio del proyecto.

**Instalar y ejecutar (Windows):**

```bash
cd gui/R5B_Lab
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

`crear_ejecutable.bat` arma un `.exe` con PyInstaller para compartirlo sin Python. Sin puerto abierto, la app funciona en simulación con la misma cinemática del firmware.

| Pantalla | Qué hace |
|---|---|
| Inicio | Gemelo 3D, puertos COM con VID:PID, conexión y estado de la ESP32 |
| Monitoreo en vivo | Estado del firmware, JOG articular y cartesiano, cero por utillaje, modo de ensamblaje, IMU en vivo y grabación de 10–30 s (prueba de golpes), consola serial |
| Trayectorias | Figuras (máx. 130 puntos, esquinas marcadas), ir a un punto, secuencias con pausa, puntos de calibración (18 puntos), ciclo pick and place, excitación P → P + Δ → P (Paper 1) |
| Control y estimación | Modo de la corrección con IMU (Apagada / Sombra / Activa), parámetros de la ley, estado del EKF en vivo; prealimentación y carga (con el firmware 1.4) |
| Resultados | Cada corrida (real o simulada) con sus gráficas, desviación y CSV |

**Registro de corridas:** cada corrida del robot real se guarda sola en `Descargas\Resultados_BNO055\etapa1\<fecha>_<nombre>\` con `imu.csv` (IMU a 100 Hz, ángulos de los motores y estados del EKF), `telemetria.csv` e `info.json` (configuración, versión del firmware, corrección, carga). Son los archivos que leen los scripts de análisis.

## Etapas de la IMU (`imu/`)

| Etapa | Carpeta | Qué hace |
|---|---|---|
| 0 | `imu/etapa0_bno055/` | Programa independiente para la BNO055 (registros, sin librerías) y script para registrar 10 min en reposo y calcular sesgo, ruido, Allan y la matriz R del EKF |
| 1 | `imu/etapa1/` | Compara el giro medido con el que predice la cinemática en las corridas de la app; analiza golpes (frecuencia, amortiguamiento, asentamiento), inclinación y calibración cinemática con la IMU |
| 2 | `imu/etapa2_modelo_ekf/` | Modelo dinámico M(q) + servo, identificación, EKF y LQR offline, simulación del lazo, seguimiento del marcador y métricas con video |

```bash
pip install pyserial numpy scipy matplotlib
python imu/etapa0_bno055/bno055_etapa0.py puertos
python imu/etapa0_bno055/bno055_etapa0.py registrar --puerto COM4 --minutos 10 --salida reposo_servos_on
python imu/etapa1/etapa1_analizar.py
```

## Análisis del Paper 1 (`paper1-tecnologicas/scripts`)

`python scripts/run_all.py` rehace todo desde los datos crudos: calibración con video, respuesta libre, identificación, ANOVA, Wilcoxon, tablas y figuras. Ver [paper1-tecnologicas/README.md](../paper1-tecnologicas/README.md).

## Simulación (`simulacion/matlab`)

Ejecutar `main.m` en MATLAB (con la carpeta en el path).

| Archivo | Función |
|---|---|
| `main.m` | Parámetros del robot (geometría v3 y masas medidas), puntos de la trayectoria, límites y llamada a la simulación |
| `ik5bar.m` | Cinemática inversa con los 4 modos de ensamblaje (`config` 1–4) |
| `forward_kinematics_5bar.m` | Cinemática directa (intersección de circunferencias) |
| `jacobiano_analitico.m` | J_P, J_θ y J (recibe el punto P para usar la rama correcta) |
| `inverse_jacobian.m` | θ̇ = J⁻¹ṗ con inversa, pseudoinversa o mínimos cuadrados amortiguados |
| `trayectoria_quintica_5bar.m` | Trayectoria quíntica entre dos puntos (pick → place) |
| `trayectoria_multi_5bar.m` | Trayectoria quíntica por varios puntos, abierta o cerrada |
| `animate_trajectory_5bar.m` | Animación del robot y gráficas de θ1, θ2, \|v\|, \|a\| |

## Software de los servos (JMC)

Los servos se configuran por RS232 con **JmcServoPcControl 1.7.6** (Windows) y la utilidad **LoadJMCServoData**. Son programas del fabricante y no se suben al repositorio. Alternativas de código abierto: [iHSV-Servo-Tool](https://github.com/robert-budde/iHSV-Servo-Tool) y [JMC-Servo-Configuration](https://github.com/MikesMachines/JMC-Servo-Configuration). La sintonía congelada está en [02-hardware.md](02-hardware.md#sintonía-congelada-13-nov-2026).

## Convenciones

- Unidades SI en el firmware (m, rad, s); la app y los comandos `JOG`/`SETHOME` usan grados; la app muestra mm.
- La geometría vive en `kinematics_5bar.h`; la app la pide con `GEOM`.
- La numeración de los modos de ensamblaje es la misma en firmware, app y MATLAB.
- Los datos de la IMU en `imu.csv` van en unidades físicas (°/s, m/s², grados); la línea `I` del firmware va en enteros escalados (ver `robot_5bar_esp32.ino`).

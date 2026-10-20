# 04 · Software

| Módulo | Ruta | Lenguaje | Corre en |
|---|---|---|---|
| Firmware del robot | `firmware/robot_5bar_esp32/` | C++ (Arduino) | ESP32-S3 |
| Núcleo cinemático | `firmware/robot_5bar_esp32/kinematics_5bar.h` | C++ (solo `<cmath>`) | ESP32-S3 (y tests en PC) |
| Interfaz de control (GUI) | `gui/robot_5bar_gui.py` | Python 3 | PC |
| Simulación de trayectorias | `simulacion/matlab/` | MATLAB | PC |
| Lectura de la IMU (etapa 0) | `imu/etapa0/imu_etapa0/imu_etapa0.ino` | C++ (Arduino) | ESP32-S3 |
| Registro y análisis de la IMU | `imu/etapa0/etapa0_imu.py` | Python 3 | PC |
| Configuración de los servos | JMC `JmcServoPcControl` 1.7.6 (no incluido) | — | PC (Windows, RS232) |

## Firmware del robot (`robot_5bar_esp32`)

**Requisitos:** Arduino IDE con el core Arduino-ESP32, placa *ESP32S3 Dev Module*, librería **FastAccelStepper** (gin66) desde el Library Manager. `kinematics_5bar.h` debe estar en la misma carpeta que el `.ino`.

**Qué hace:**
- Recibe comandos de texto por USB (115200 bps).
- Valida los puntos (alcance, singularidad, límites articulares).
- Interpola trayectorias rectas con perfil quíntico y calcula la cinemática inversa a 250 Hz.
- Genera los pulsos coordinados de los dos ejes con FastAccelStepper (por hardware, sin bloquear el `loop`).
- Envía telemetría (`STATE`) a 20 Hz.
- Fija el cero por utillaje (`SETHOME`) y permite mover un eje a mano (`JOG`).

**Parámetros que se ajustan en el código:**

| Constante | Archivo | Valor actual | Qué es |
|---|---|---|---|
| `L1, L2, L3, L4, D` | `kinematics_5bar.h` | 0,090 / 0,165 / 0,165 / 0,090 / 0,155 m | Geometría (actualizar a v2) |
| `STEPS_PER_REV` | `kinematics_5bar.h` | 8000 | Debe ser igual a la resolución configurada en el servo |
| `SIN_MIN` | `kinematics_5bar.h` | 0,15 | Margen de singularidad |
| `PIN_PUL1/DIR1/PUL2/DIR2` | `.ino` | 4 / 5 / 6 / 7 | Pines |
| `DIR_SIGN1/2` | `.ino` | +1 / +1 | Invertir si un motor gira al revés |
| `TH1_HOME_DEG / TH2_HOME_DEG` | `.ino` | 90° / 90° | Ángulos que deja el utillaje |
| `TH1_MIN…TH2_MAX` | `.ino` | [10, 250]° / [−70, 250]° | Límites articulares blandos |
| `g_Vmax / g_Amax` | `.ino` | 0,10 m/s / 0,20 m/s² | Velocidad y aceleración iniciales del efector |

El protocolo de comandos está en [05-arquitectura-software.md](05-arquitectura-software.md#protocolo-serie).

## Interfaz de control (`gui/robot_5bar_gui.py`)

**Instalar y ejecutar:**

```bash
pip install -r gui/requirements.txt
python gui/robot_5bar_gui.py
```

**Funciones:**
- Conexión al puerto COM; al conectar envía `PING` y `GEOM` y adopta la geometría del firmware.
- Vista del espacio de trabajo con el robot dibujado en tres capas: pose de cero (gris), vista previa al pasar el ratón (naranja punteado) y pose real por telemetría (azul/verde).
- Botón de **PARO**.
- Calibración del cero: ángulos del utillaje, *jog* por eje y **Fijar CERO**.
- Deslizadores de velocidad y aceleración del efector (`SETV`).
- *Ir a coordenada* escribiendo X, Y o haciendo clic en el área.
- *Trazar figura* (círculo, cuadrado, triángulo, estrella) con centro y tamaño; se valida que todos los puntos sean alcanzables y se envían como `PATH` (máximo 130 puntos).

**Mejora en curso:** el equipo está diseñando su propia aplicación con estilo más profesional (tema claro, pantalla de inicio con conexión serial y botón *Comencemos*, monitoreo con movimiento manual/cero y vista 3D, trayectorias con figuras e ir a un punto), por recomendación del docente. Primero se hace la parte visual y después se une con el código actual.

## Simulación (`simulacion/matlab`)

Ejecutar `main.m` en MATLAB (con la carpeta en el path).

| Archivo | Función |
|---|---|
| `main.m` | Parámetros del robot, puntos de la trayectoria, límites y llamada a la simulación |
| `ik5bar.m` | Cinemática inversa con los 4 modos de ensamblaje (`config` 1–4) |
| `forward_kinematics_5bar.m` | Cinemática directa (intersección de circunferencias) |
| `jacobiano_analitico.m` | J_P, J_θ y J |
| `inverse_jacobian.m` | θ̇ = J⁻¹ṗ con inversa, pseudoinversa o mínimos cuadrados amortiguados |
| `trayectoria_quintica_5bar.m` | Trayectoria quíntica entre dos puntos (pick → place) |
| `trayectoria_multi_5bar.m` | Trayectoria quíntica por varios puntos, abierta o cerrada |
| `animate_trajectory_5bar.m` | Animación del robot y gráficas de θ1, θ2, \|v\|, \|a\| |

## IMU – etapa 0 (`imu/etapa0`)

Firmware independiente del robot que lee la MPU6050 a 200 Hz, más un script de Python para registrar y analizar (sesgos, densidad de ruido, desviación de Allan, deriva térmica). Guía paso a paso en [imu/etapa0/README.md](../imu/etapa0/README.md).

```bash
pip install pyserial numpy matplotlib
python imu/etapa0/etapa0_imu.py puertos
python imu/etapa0/etapa0_imu.py registrar --puerto COM5 --minutos 10 --salida reposo_servos_off.csv
python imu/etapa0/etapa0_imu.py analizar reposo_servos_off.csv
```

## Software de los servos (JMC)

Los servos se configuran por RS232 con **JmcServoPcControl 1.7.6** (Windows) y la utilidad **LoadJMCServoData**. Son programas del fabricante y no se suben al repositorio. Alternativas de código abierto: [iHSV-Servo-Tool](https://github.com/robert-budde/iHSV-Servo-Tool) y [JMC-Servo-Configuration](https://github.com/MikesMachines/JMC-Servo-Configuration).

## Convenciones

- Unidades SI en todo el código (m, rad, s); la GUI y los comandos `JOG`/`SETHOME` usan grados.
- La geometría vive en `kinematics_5bar.h`; los demás programas la copian o la piden al firmware.
- La numeración de los modos de ensamblaje es la misma en firmware, GUI y MATLAB.

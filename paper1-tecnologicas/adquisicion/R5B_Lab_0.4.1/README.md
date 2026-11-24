# R5B Lab · Interfaz del robot paralelo de 5 barras

Aplicación de escritorio en Python (PySide6) para el proyecto de Técnicas de Control.

## Cómo ejecutarla (Windows + VS Code)

1. Abra la carpeta `R5B_Lab` en VS Code (Archivo → Abrir carpeta).
2. Abra una terminal en VS Code (Terminal → Nueva terminal) y cree el entorno virtual:

   ```
   python -m venv .venv
   .venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Ejecute la aplicación:

   ```
   python main.py
   ```

Si VS Code pregunta qué intérprete usar, elija el de `.venv`.

## Compartir la aplicación (ejecutable de Windows)

Sus compañeros no necesitan Python: basta con el ejecutable.

1. En su PC, haga doble clic en `crear_ejecutable.bat` (tarda 1 a 3 minutos la primera vez).
2. Se crea `dist\R5B_Lab_Windows.zip`. Compártalo por Drive, OneDrive, WhatsApp, etc.
3. Quien lo reciba: descomprime el .zip, entra a la carpeta `R5B Lab` y abre `R5B Lab.exe`.
   No se debe sacar el .exe de esa carpeta (necesita la carpeta `_internal` a su lado).

La primera vez Windows puede mostrar "Windows protegió su PC", porque el programa no está firmado
digitalmente: pulse **Más información → Ejecutar de todas formas**.

**Opción automática con GitHub:** si suben el proyecto a un repositorio de GitHub, el archivo
`.github/workflows/ejecutable-windows.yml` genera el .zip solo cada vez que suben cambios
(pestaña *Actions* → última ejecución → *Artifacts*). Si crean una etiqueta de versión (`v0.2.0`),
el .zip queda publicado en *Releases*, con un enlace de descarga para todo el equipo.

## Usar el robot real (versión 0.3)

**Geometría actual:** proximales L1 = L4 = 120 mm, distales L2 = L3 = 180 mm, distancia entre ejes D = 150 mm.
Al conectar, la app toma la geometría que responde el firmware al comando `GEOM`, así que
`kinematics_5bar.h` debe tener estas mismas medidas (en metros: 0.12, 0.18, 0.18, 0.12, 0.15).
Con el cero en 90° / 90° el efector queda en (75, 283.6) mm.

La app habla el protocolo del firmware `robot_5bar_esp32.ino` (115200 bps). Antes de conectar,
cierre el Monitor Serie del IDE de Arduino (solo un programa puede usar el puerto).

1. **Inicio:** elija el puerto de la ESP32-S3 (el USB nativo aparece como *USB JTAG/serial debug unit
   [303A:...]*), pulse **Conectar** y espere "Robot conectado". La app pide `PING`, `GEOM` y `MODE`
   y toma la geometría del firmware. Si queda "esperando a la ESP32", pulse RESET en la placa.
2. **Monitoreo → Cero del robot:** con JOG articular acerque los brazos, encaje el utillaje,
   ponga sus ángulos (90° / 90° por defecto) y pulse **Fijar cero** (`SETHOME`). Retire el utillaje.
3. Ya puede mover en cartesiano (cada clic = `GOTO` al punto vecino), cambiar el **modo de
   ensamblaje** (`MODE n`) y usar **Trayectorias**.
4. **Parada de emergencia** o tecla **Esc** = `STOP` y bloquea la app hasta volver a habilitar.

| Acción en la app | Comando al firmware |
|---|---|
| Conectar | `PING`, `GEOM`, `MODE`, `STATUS` (luego telemetría `STATE` a 20 Hz) |
| JOG articular ± | `JOG <1/2> <grados>` (funciona sin cero) |
| JOG cartesiano / Ir a un punto / clic en el mapa | `GOTO x y` (m) |
| Fijar cero | `SETHOME θ1 θ2` |
| Modo de ensamblaje | `MODE n` |
| Figura | `SETV vmax amax`, `PBEGIN n`, `PADD ...` (8 puntos por línea, con confirmación), `PRUN` |
| Secuencia | un `GOTO` por punto, esperando `EVT DONE` |
| Parada / Detener | `STOP` |
| Conectar (con IMU) | `IMU 1` → líneas `I t q1 q2 ax ay az gx gy gz rumbo roll pitch cal` a 100 Hz |

### IMU BNO055 (versión 0.3, etapa 1)

Con el firmware que incluye `imu_bno055.h`, al conectar la app pide `IMU 1` y la ESP32 envía una línea
`I ...` a 100 Hz con la IMU y los ángulos de los motores del mismo instante. La tarjeta de conexión muestra
"· IMU" cuando llegan datos. Cada corrida del robot real se graba sola, desde 0.5 s antes hasta 2.5 s después,
en `Descargas\Resultados_BNO055\etapa1\<fecha>_<nombre>\` (`imu.csv`, `telemetria.csv`, `info.json`).
En **Resultados → IMU · gz** se compara el gz medido con el predicho para los dos eslabones distales.
El análisis completo lo hace `etapa1_analizar.py`.

**Versión 0.4.0 (para el firmware 1.4):** en **Control y estimación** hay una sección nueva,
**Prealimentación por modelo**: *Apagada / Encendida* se envía al elegirla (como el modo) y la **ganancia** con
**Aplicar**. El campo **Carga en el efector (g)** no se envía al robot: se anota en cada corrida
(`info.json → carga_g`) y en el nombre de la carpeta. Las carpetas terminan según la condición: sin sufijo
(apagada), `_ff`, `_conIMU`, `_ffIMU`, `_sombra`, más `_c150g` si hay carga. Con el firmware 1.4, `imu.csv`
agrega el plan y lo que se sumó a los pulsos (`q1_plan_deg`, `q2_plan_deg`, `ff1_deg`, `ff2_deg`,
`dq1_aplicada_deg`, `dq2_aplicada_deg`). La barra de conexión dice "· prealimentación" y "· carga 150 g".
Con el firmware 1.3 todo sigue funcionando; la prealimentación aparece deshabilitada.

**Versión 0.3.6:** en Resultados, al elegir una corrida, la línea de abajo dice su modo de corrección
(apagada / sombra / activa, con α, Δq máximo y lavado) y si se desactivó sola; antes quedaba el aviso de la
corrida anterior. `info.json` guarda la versión del firmware aunque la app se conecte con la ESP32 ya encendida.

**Versión 0.3.5 (para el firmware 1.3):** la pantalla **Control y estimación** ya maneja la corrección con
IMU del firmware 1.3 (comandos `CTL`). Modo **Apagada** (igual que el firmware 1.x, corridas "sin IMU"),
**Sombra** (el EKF y la ley corren y se registran, pero no se aplican) o **Activa** (la corrección Δq se suma
a los pulsos). El modo se envía al elegirlo; los parámetros (α, zona muerta, Δq máximo, lavado) con
**Aplicar**. La tarjeta "Estado en el robot" muestra en vivo q̂ − q, ω̂, Δq, la innovación y el tiempo de
cálculo del EKF en la ESP32. La barra de conexión indica "· EKF en sombra" o "· corrección IMU activa".
Cada corrida guarda en `info.json` la configuración vigente (`correccion_imu`, con los eventos de apagado
automático) y su carpeta termina en `_sombra` o `_conIMU`. `imu.csv` agrega las columnas `q1_ekf_deg`,
`q2_ekf_deg`, `w1_ekf_dps`, `w2_ekf_dps`, `d1_mNm`, `d2_mNm`, `dq1_ley_deg`, `dq2_ley_deg`, `inov_gz_dps`,
`ctl_modo`, `ekf_valido`, `ctl_aplicando`. En Resultados: curva "Estimado por el EKF" en Trayectoria x–y,
desviación con el EKF, y la pestaña **EKF · Δq**. Con el firmware 1.x la app funciona igual que la 0.3.4.

**Versión 0.3.4:** en Resultados, el recorrido, los ángulos, la velocidad y la desviación se calculan con los
ángulos que llegan con cada muestra de la IMU (100 Hz, según los pulsos emitidos) en lugar de la telemetría
STATE (20 Hz con la IMU activa), que al dibujarse "cortaba" las esquinas y parecía no pasar por los puntos.
La curva se llama ahora "Pulsos emitidos (comando)": es lo que se le mandó al robot, no su posición real.
Compatible con el firmware 2.0 (pulsos sincronizados) y con el anterior.

**Versión 0.3.3:** en Trayectorias → Secuencia está el botón **Ciclo pick and place**: 10 ciclos entre
(0, 180) mm (Tomar) y (150, 250) mm (Soltar), pausa de 2 s en cada punto, 100 mm/s y 1000 mm/s². La corrida
se guarda como "Pick and place 10 ciclos". La garra todavía no se maneja desde la ESP32; la columna "Acción"
queda como registro.

**Versión 0.3.2:** en Trayectorias → Secuencia hay una **pausa en cada punto** (0 a 10 s) y el botón
**Puntos de calibración**, que carga una cuadrícula de ~18 puntos para la calibración cinemática con la IMU
(pausa de 2 s, 50 mm/s, 200 mm/s²). La app guarda hasta 6 min de IMU por corrida.

**Versión 0.3.1:** en Monitoreo, la tarjeta **IMU · grabar** muestra la IMU en vivo (gz, |a|, roll, pitch y
calibración) y graba 10, 20 o 30 s con el robot quieto, para la prueba de golpes. La grabación se guarda como
una corrida más, con `"tipo": "grabacion"` y la postura (`x_mm`, `y_mm`, `theta1_deg`, `theta2_deg`) en `info.json`.

Sin puerto abierto la app funciona en **simulación** con la misma cinemática (`kinematics_5bar.h`
traducido a Python en `r5b/kinematics.py`).

**Aún no disponible en el firmware** (la app lo indica): ventosa / bomba de vacío, ENABLE de los
servos y movimiento articular a un punto. En las secuencias, las acciones Tomar / Soltar se omiten
con el robot real.

## Pantallas

| Pantalla | Qué hace |
|---|---|
| Inicio | gemelo 3D girando, puertos COM con VID:PID, conexión y estado de la ESP32 |
| Monitoreo en vivo | estado del firmware, JOG articular y cartesiano, cero por utillaje, modo de ensamblaje, gemelo 3D con la postura de la telemetría, consola serial |
| Trayectorias | figuras (máx. 130 puntos, esquinas marcadas donde frena), ir a un punto, secuencias; Vmax / Amax; validación con la misma guarda de singularidad del firmware |
| Control y estimación | corrección con IMU del firmware 1.3: modo (apagada / sombra / activa), parámetros de la ley y estado del EKF en vivo |
| Resultados | registro de cada corrida (real o simulada), desviación respecto a la referencia, gráficas y CSV |

## Estructura

```
main.py                  arranque de la aplicación
r5b/config.py            geometría, colores y fuentes
r5b/theme.py             hoja de estilos (tema claro)
r5b/kinematics.py        cinemática directa / inversa
r5b/paths.py             figuras y validación de recorridos
r5b/robot_state.py       estado del robot y ejecución de movimientos
r5b/robot_link.py        comunicación serial con la ESP32
r5b/robot3d.py           gemelo 3D (QPainter)
r5b/workspace_map.py     mapa del espacio de trabajo
r5b/screens/             una pantalla por archivo
```

## Fuentes (opcional)

El diseño usa IBM Plex Sans e IBM Plex Mono. Si no están instaladas, la app usa Segoe UI y Consolas.
Para el aspecto exacto, descárguelas de Google Fonts y copie los `.ttf` en `r5b/assets/fonts/`.

## Novedades de la 0.4.1 (22 nov 2026)
- Trayectorias → Secuencia: **Excitación · Δ / dirección / ciclos → "Cargar excitación aquí"**. Desde la
  postura actual P arma N ciclos P → P + Δ → P con pausa de 2 s (ensayo de excitación comandada del Paper 1).
  La corrida se guarda como `..._Excitacion_5_ciclos_75_280_d12_menosy` (postura y Δ en el nombre).
- `crear_ejecutable.bat` usa el Python del entorno `.venv` (`python -m PyInstaller`) y el lanzador `py`,
  para no tomar el Python de MSYS2.

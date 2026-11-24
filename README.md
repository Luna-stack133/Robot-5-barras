# Robot paralelo de 5 barras

Plataforma robótica paralela de bajo costo tipo **SCARA paralelo de 3 GDL**: un mecanismo plano de cinco barras para X-Y y un eje lineal Z en el efector, con garra. El plano X-Y lo mueven dos servomotores AC integrados **JMC iHSV57-30-18-36** comandados por pulso/dirección desde una **ESP32-S3**. Una **IMU BNO055** montada dentro del eslabón distal derecho mide el giro del eslabón y alimenta un EKF que corre en la misma ESP32. Se usa como banco de pruebas para identificación dinámica, control con realimentación inercial y pick and place.

**Universidad Nacional de Colombia — Ingeniería Mecatrónica**
Autores: Luna Valentina Niebles Gil · Isaías Lazaro Charry · Roiman Cardenas Rosado


> **Estado al 24 de noviembre de 2026**
> - Geometría v3: proximales 120 mm, distales 180 mm, separación entre ejes 150 mm; efector con torre Z y garra; cableado interno.
> - Firmware **1.3** (EKF y corrección con IMU, arranca apagada) y app de escritorio **R5B Lab 0.4.1** (PySide6).
> - Sintonía de los drivers congelada desde el 13 nov; mantenimiento de acoples el 22 nov; calibración cinemática con video: 1.15–1.17 mm RMS.
> - **Paper 1** (identificación dinámica, TecnoLógicas): campaña S1–S4 terminada, análisis y manuscrito listos.
> - **Paper 2** (control con realimentación inercial, Ingeniería e Investigación): en preparación. Primera sesión con/sin IMU hecha el 15 nov.
> - En diseño: reductor planetario 6:1 impreso para bajar la relación de inercia.

---

## Secciones del repositorio

El repositorio está organizado por secciones. La **plataforma** es común a todo el proyecto; cada **artículo** tiene su propia carpeta autocontenida (datos crudos, scripts, resultados, figuras y un README para correr todo de principio a fin).

| Sección | Carpeta | Contenido |
|---|---|---|
| Plataforma | `firmware/`, `gui/`, `imu/`, `simulacion/`, `hardware/`, `docs/` | Firmware de la ESP32-S3, app R5B Lab, etapas de la IMU (caracterización, IMU frente a cinemática, modelo y EKF), simulación, hardware y documentación |
| Paper 1 — TecnoLógicas | [`paper1-tecnologicas/`](paper1-tecnologicas/) | *Experimental Identification of Configuration-Dependent Dynamics and Actuation Constraints in a Low-Cost Parallel-SCARA Robot*: datos de las sesiones S1–S4, software de adquisición usado (firmware 1.3, app R5B Lab 0.4.1), análisis, figuras y respuestas a las preguntas del docente |
| Paper 2 — Ingeniería e Investigación | [`paper2-ingenieria-e-investigacion/`](paper2-ingenieria-e-investigacion/) | Control con realimentación inercial (EKF + IMU). En preparación |

## Documentación

| Área | Documento | Qué contiene |
|---|---|---|
| Fabricación | [docs/01-fabricacion.md](docs/01-fabricacion.md) | Geometría (versiones), eslabones v3 con cableado interno, efector con eje Z y garra, utillaje de cero, mantenimiento, mejoras planeadas |
| Hardware | [docs/02-hardware.md](docs/02-hardware.md) | Servos iHSV57 y sus parámetros, ESP32-S3, buffer 74HCT14, BNO055, pines, cableado |
| Control | [docs/03-control.md](docs/03-control.md) | Cinemática, singularidades, trayectoria continua, modelo identificado, EKF y ley de corrección con IMU |
| Software | [docs/04-software.md](docs/04-software.md) | Firmware 1.3 / 1.4, app R5B Lab, scripts de la IMU, simulación; cómo compilar y ejecutar |
| Arquitectura de software | [docs/05-arquitectura-software.md](docs/05-arquitectura-software.md) | Componentes, protocolo serie, máquina de estados, tareas de la ESP32 y flujo de datos |
| Investigación | [docs/06-investigacion.md](docs/06-investigacion.md) | Estado del arte, brecha, papers en curso y planteamiento por asignatura |
| Bitácora | [docs/bitacora.md](docs/bitacora.md) | Avances, decisiones y hallazgos con fecha |
| Pendientes | [docs/pendientes.md](docs/pendientes.md) | Lo que falta hacer, documentar o corregir |

## Estructura del repositorio

```
Robot-5-barras/
├── firmware/
│   ├── robot_5bar_esp32/              Firmware 1.3 (en uso): trayectorias, IMU BNO055, EKF y corrección
│   │   ├── robot_5bar_esp32.ino
│   │   ├── kinematics_5bar.h          Núcleo cinemático: única fuente de verdad de la geometría
│   │   ├── imu_bno055.h · control_imu.h · ekf_5b.h · lqr_tabla.h
│   │   └── LEEME_v1.3.md
│   └── v1.4_prealimentacion/          Firmware 1.4 (prealimentación por modelo; aún no probado en el robot)
├── gui/
│   └── R5B_Lab/                       App de escritorio R5B Lab 0.4.1 (Python + PySide6)
├── imu/
│   ├── etapa0_bno055/                 Caracterización de la BNO055 en reposo (firmware + análisis)
│   ├── etapa1/                        IMU frente a la cinemática, golpes, calibración con la IMU
│   └── etapa2_modelo_ekf/             Modelo dinámico, identificación, EKF offline, video
├── simulacion/
│   └── matlab/                        Cinemática, Jacobiano y trayectorias quínticas (main.m)
├── hardware/
│   ├── cad/                           Modelos CAD / STL (por subir)
│   └── datasheets/                    Enlaces a manuales de los componentes
├── docs/                              Documentación por áreas
├── paper1-tecnologicas/               Paper 1: datos, adquisición, scripts, resultados, figuras
│   ├── adquisicion/                   Firmware 1.3 y app R5B Lab 0.4.1 usados en las sesiones
│   ├── data/                          raw/ (S1–S4, video_tracking), runs.csv, exclusions.csv
│   ├── scripts/                       Análisis completo (run_all.py), seguimiento de video, modelo
│   ├── results/  figures/             Salidas numéricas y figuras 1–6
│   └── docs/                          Notas de sesiones, fotos, ensayos opcionales
└── paper2-ingenieria-e-investigacion/ Paper 2 (en preparación)
```

## Inicio rápido

1. **Firmware:** abrir `firmware/robot_5bar_esp32/robot_5bar_esp32.ino` en Arduino IDE, placa *ESP32S3 Dev Module*, instalar la librería **FastAccelStepper** (gin66) y cargar. No usar *Erase All Flash* (borra la calibración guardada de la IMU).
2. **App:** en `gui/R5B_Lab/`, `python -m venv .venv`, activar el entorno, `pip install -r requirements.txt` y `python main.py`. También se puede crear un ejecutable con `crear_ejecutable.bat`.
3. En la app: **Inicio** → elegir el puerto de la ESP32-S3 → **Conectar**. En **Monitoreo**: acercar los brazos con JOG, poner el utillaje → **Fijar cero** (90°/90°) → retirar el utillaje. Mover con *Ir a un punto*, *Figuras* o *Secuencias*.
4. La corrección con IMU arranca **Apagada**. Se cambia en **Control y estimación** (Apagada / Sombra / Activa).

Detalles y advertencias de seguridad en [docs/04-software.md](docs/04-software.md) y [docs/02-hardware.md](docs/02-hardware.md).

## Video y fotos

Las fotos del montaje van en [`docs/img/`](docs/img/). La fotografía del robot completo para el Paper 1 va en `paper1-tecnologicas/docs/fotos/`.

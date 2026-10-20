# Robot paralelo de 5 barras

Plataforma robótica paralela plana de 2 GDL y bajo costo, accionada por dos servomotores AC integrados **JMC iHSV57-30-18-36** comandados por pulso/dirección desde una **ESP32-S3**. Se usa como banco de pruebas para cinemática, generación de trayectorias, control y fusión de sensores (IMU), y es la base de un pick and place con gripper y, más adelante, control servo-visual.

**Universidad Nacional de Colombia — Ingeniería Mecatrónica**
Equipo: Luna Valentina Niebles Gil · Isaías Lazaro Charry · Roiman (gripper)
Asignaturas: Técnicas de Control · Robótica · Práctica Académica Especial (PAE 2027-1)

> Estado al 20 de octubre de 2026: el robot se mueve por trayectorias cartesianas desde una GUI en Python, con cero por utillaje. La IMU está en la etapa 0 (caracterización). El gripper, los encoders externos y el lazo con IMU están en planeación.

---

## Documentación

| Área | Documento | Qué contiene |
|---|---|---|
| Fabricación | [docs/01-fabricacion.md](docs/01-fabricacion.md) | Geometría (versiones), montaje, utillaje de cero, efector final, mejoras planeadas |
| Hardware | [docs/02-hardware.md](docs/02-hardware.md) | Servos iHSV57, ESP32-S3, buffer 74HCT14, cableado y pines, IMU, alimentación |
| Control | [docs/03-control.md](docs/03-control.md) | Cinemática, singularidades, perfil quíntico, esquema actual y esquema EKF + IMU planeado |
| Software | [docs/04-software.md](docs/04-software.md) | Firmware, GUI, simulación MATLAB, herramientas IMU y JMC; cómo compilar y ejecutar |
| Arquitectura de software | [docs/05-arquitectura-software.md](docs/05-arquitectura-software.md) | Componentes, protocolo serie, máquina de estados, flujos y arquitectura objetivo |
| Investigación | [docs/06-investigacion.md](docs/06-investigacion.md) | Estado del arte, brecha encontrada, planteamiento por asignatura y PAE |
| Bitácora | [docs/bitacora.md](docs/bitacora.md) | Avances, decisiones y hallazgos con fecha |
| Pendientes | [docs/pendientes.md](docs/pendientes.md) | Lo que falta documentar o corregir |

## Estructura del repositorio

```
robot-5-barras/
├── firmware/
│   └── robot_5bar_esp32/      Firmware del robot (ESP32-S3, Arduino)
│       ├── robot_5bar_esp32.ino
│       └── kinematics_5bar.h  Núcleo cinemático: única fuente de verdad de la geometría
├── gui/
│   └── robot_5bar_gui.py      Interfaz de control (Python + Tkinter + Matplotlib)
├── simulacion/
│   └── matlab/                Cinemática, Jacobiano y trayectorias quínticas (main.m)
├── imu/
│   └── etapa0/                Lectura y caracterización de la MPU6050 (firmware + análisis)
├── hardware/
│   ├── cad/                   Modelos CAD / STL (por subir)
│   └── datasheets/            Enlaces a manuales de los componentes
└── docs/                      Documentación por áreas
```

## Inicio rápido

1. **Firmware:** abrir `firmware/robot_5bar_esp32/robot_5bar_esp32.ino` en Arduino IDE, placa *ESP32S3 Dev Module*, instalar la librería **FastAccelStepper** (gin66) y cargar.
2. **GUI:** `pip install -r gui/requirements.txt` y luego `python gui/robot_5bar_gui.py`.
3. Conectar al puerto COM de la ESP32 → poner el utillaje → **Fijar CERO** → retirar el utillaje → mover con *Ir a coordenada* o *Trazar figura*.

Detalles y advertencias de seguridad en [docs/04-software.md](docs/04-software.md) y [docs/02-hardware.md](docs/02-hardware.md).

## Video y fotos

Las fotos del montaje van en [`docs/img/`](docs/img/). *(Pendiente: subir fotos del robot, del cableado y del utillaje.)*

# 06 · Investigación y planteamiento

## Estado del arte (resumen)

Se hicieron tres búsquedas en Scopus (363 registros en total):

| Búsqueda | Registros | Hallazgo principal |
|---|---|---|
| Mecanismos de 5 barras | 213 (1967–2026) | Tema activo y creciente; pocas aplicaciones de pick and place industrial (5) |
| Robots paralelos + IMU | 106 | Solo 4 cruzan con 5 barras |
| Servo AC integrado / closed-loop stepper | 44 | Ninguno trata un mecanismo de 5 barras |

**Antecedentes más cercanos con 5 barras + IMU**, todos con la IMU usada de forma pasiva (fuera del lazo de control):
- Pulloquinga, Ceccarelli, Mata & Valera (2024), *Actuators* 13(5):168 — 5R educativo de pick and place; MPU6050 en el efector solo para detectar singularidades. https://doi.org/10.3390/act13050168
- Seo & Kim (2021), *Sensors* 21(5):1726 — máquina pasiva de marcha analizada con IMUs. https://doi.org/10.3390/s21051726
- Zapatero-Gutiérrez, Castillo-Castañeda & Laribi (2022), *Robotica* — rehabilitación de dedos, motores DC; la IMU solo valida. https://doi.org/10.1017/S0263574722001217

**Robots de 5 barras con servos industriales:** Zhang et al. (2024, Delta ASDA-A2, PID), Campos et al. (2010, DexTAR, direct drive Kollmorgen).

## Brecha identificada

No se encontró ningún robot de 5 barras que combine:
1. servomotores AC integrados de bajo costo comandados por pulsos,
2. una IMU usada como **realimentación activa** dentro del lazo de control, y
3. la arquitectura paralela de 5 barras.

Cada pieza tiene precedentes por separado; la combinación no.

## Planteamiento por asignatura

### Técnicas de Control (prof. Francisco Javier Burgos Flórez)

- **Pregunta:** ¿en qué medida mejora el seguimiento de trayectoria de un robot de 5 barras con servos AC integrados al incorporar un EKF con fusión activa de IMU dentro del lazo de control?
- **Objetivo general:** diseñar, implementar y validar experimentalmente un esquema de control de trayectoria con estimación por EKF + IMU, y medir la mejora frente al mismo esquema sin la IMU activa.
- **Alcance decidido:** no se implementa un PD/CTC clásico aparte para comparar.

### Robótica (prof. Cesar Augusto David Meza)

Enfoque en la cinemática del robot y en el gripper como efector final.

### PAE 2027-1

**Título:** *Desarrollo de una celda robótica didáctica de pick and place con control servo-visual basado en la fusión de cámara e IMU, sobre un robot paralelo de cinco barras.*

Integrantes: Luna, Isaías y Roiman. Este semestre: plataforma de bajo costo (trayectorias y pick and place), gripper adaptativo y control. Siguiente semestre: integración IMU + cámara para pick and place y, como línea futura, aprendizaje por refuerzo.

## Referencias principales

- Chaumette, F., & Hutchinson, S. (2006). Visual servo control. I. Basic approaches. *IEEE Robotics & Automation Magazine*, 13(4), 82–90. https://doi.org/10.1109/MRA.2006.250573
- Merlet, J.-P. (2006). *Parallel robots* (2nd ed.). Springer. https://doi.org/10.1007/1-4020-4133-0
- Mora, S. N., & Martínez Santos, J. C. (2015). Visual servoing of a five-bar linkage mechanism. *IEEE CCAC*. https://doi.org/10.1109/CCAC.2015.7345205
- Pulloquinga, J. L., Ceccarelli, M., Mata, V., & Valera, Á. (2024). *Actuators*, 13(5), 168. https://doi.org/10.3390/act13050168
- Yamtuan, K., Radomngam, T., & Prempraneerach, P. (2023). Visual servo kinematic control of delta robot using YOLOv5 algorithm. *Journal of Robotics and Control*, 4(6), 818–831. https://doi.org/10.18196/jrc.v4i6.19102
- Zhang, Hou, Li, Qin, Abbas & Siddique (2024). *Proc. IMechE Part C*, 238(13). https://doi.org/10.1177/09544062231224525
- JMC. *iHSV Integrated AC Servo Motor Manual*. https://www.jmc-motor.com/uploads/20240730/1cf02d411ba42d6a61bb4df55b64b835.pdf

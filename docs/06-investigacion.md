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

**Fusión de acelerómetros con encoders en robots seriales industriales** (hay que citarla y diferenciarse): Axelsson, Karlsson & Norrlöf (ABB IRB4600); Chen & Tomizuka, *IEEE/ASME TMech* 19(2):697–706, 2014.

**Identificación modal dependiente de la postura** (base del Paper 1): la lista de 15 antecedentes recientes (2023–2026) que entregó el docente está en las referencias del manuscrito del Paper 1.

## Brecha identificada

No se encontró ningún robot de 5 barras que combine:
1. servomotores AC integrados de bajo costo comandados por pulsos, sin acceso al par;
2. una IMU usada como **realimentación activa** dentro del lazo de control; y
3. la arquitectura paralela de 5 barras.

Cada pieza tiene precedentes por separado; la combinación no. Lo nuevo frente a los robots seriales: cadena cerrada, una sola IMU en un distal, servo por pulsos con lazo propio y 25–45 ms de retardo, bajo costo y aplicación de pick and place.

## Hallazgo que orienta el trabajo

La oscilación de 1,6–3,7 Hz después de cada parada viene del lazo del servo actuando contra una **relación de inercia de 105–360** (muy por encima de lo que estos servos manejan bien). La re-sintonía del driver no la quita: su margen se agota. Por eso hace falta amortiguación externa (IMU) o bajar la relación de inercia (reductor).

## Artículos en curso

Por recomendación del docente (18 nov 2026), el trabajo se divide en dos artículos, cada uno con su carpeta en este repositorio.

| | Paper 1 | Paper 2 |
|---|---|---|
| Tema | Identificación dinámica dependiente de la configuración y restricciones de accionamiento | Control con realimentación inercial (EKF + IMU) del seguimiento de trayectorias, aplicado a pick and place |
| Revista | TecnoLógicas (Publindex B) | Ingeniería e Investigación |
| Carpeta | [`paper1-tecnologicas/`](../paper1-tecnologicas/README.md) | [`paper2-ingenieria-e-investigacion/`](../paper2-ingenieria-e-investigacion/README.md) |
| Estado | Campaña S1–S4 terminada (22–23 nov); análisis reproducible; manuscrito en formato de la revista | Primera sesión con/sin IMU (15 nov); bloque definitivo por repetir después del mantenimiento |

**Paper 1 — hipótesis y resultados:**
- H1 (los modos cambian con la postura): apoyada. f_n de 1,64 a 3,73 Hz y ζ de 0,11 a 0,35 en 48 corridas (4 posturas × 3 amplitudes × 4 réplicas); postura, amplitud e interacción explican casi toda la varianza; la sesión no.
- H2 (un modelo con M(q) predice mejor que uno de inercia constante): apoyada para movimientos grandes; el modelo lineal no reproduce el ablandamiento con la amplitud.
- H3 (la oscilación no es solo mala sintonía): apoyada en parte. Con brazos bloqueados y servos apagados la oscilación desaparece, y una carga de 100 g baja la frecuencia como predice la inercia. El origen del ablandamiento (fricción, holgura o el driver) queda abierto.

## Planteamiento por asignatura

### Técnicas de Control (prof. Francisco Javier Burgos Flórez)

- **Pregunta:** ¿en qué medida mejora el seguimiento de trayectoria de un robot de 5 barras con servos AC integrados al incorporar un EKF con fusión activa de IMU dentro del lazo de control?
- **Objetivos específicos (16 nov 2026):**
  1. Diseñar el EKF basado en la IMU del distal.
  2. Diseñar las leyes de control: amortiguamiento programado con la inercia y prealimentación por modelo.
  3. Evaluar experimentalmente con y sin IMU, prealimentación y carga.
- **Alcance:** no se implementa un PD/CTC clásico aparte; se compara el mismo esquema con la IMU apagada y activa, en la misma sesión.
- **Entregables:** informe, artículo y sustentación con el robot.

### Robótica (prof. Cesar Augusto David Meza)

Enfoque en la cinemática y la dinámica del robot (inversa, directa, Jacobianos, trayectoria quíntica) y en el gripper como efector final.

### PAE 2027-1

**Título:** *Desarrollo de una celda robótica didáctica de pick and place con control servo-visual basado en la fusión de cámara e IMU, sobre un robot paralelo de cinco barras.*

Integrantes: Luna, Isaías y Roiman. Plataforma de bajo costo (trayectorias y pick and place), gripper adaptativo y control; después, integración IMU + cámara para pick and place y, como línea futura, aprendizaje por refuerzo.

## Referencias principales

- Axelsson, Karlsson & Norrlöf — estimación con acelerómetro y encoders en un ABB IRB4600 (completar la referencia). https://people.isy.liu.se/en/rt/axelsson/papers/axelssonKN12b.pdf
- Chaumette, F., & Hutchinson, S. (2006). Visual servo control. I. Basic approaches. *IEEE Robotics & Automation Magazine*, 13(4), 82–90. https://doi.org/10.1109/MRA.2006.250573
- Chen & Tomizuka (2014). *IEEE/ASME Transactions on Mechatronics*, 19(2), 697–706 (completar título y autores).
- Merlet, J.-P. (2006). *Parallel robots* (2nd ed.). Springer. https://doi.org/10.1007/1-4020-4133-0
- Mora, S. N., & Martínez Santos, J. C. (2015). Visual servoing of a five-bar linkage mechanism. *IEEE CCAC*. https://doi.org/10.1109/CCAC.2015.7345205
- Pulloquinga, J. L., Ceccarelli, M., Mata, V., & Valera, Á. (2024). *Actuators*, 13(5), 168. https://doi.org/10.3390/act13050168
- Yamtuan, K., Radomngam, T., & Prempraneerach, P. (2023). Visual servo kinematic control of delta robot using YOLOv5 algorithm. *Journal of Robotics and Control*, 4(6), 818–831. https://doi.org/10.18196/jrc.v4i6.19102
- Zhang, Hou, Li, Qin, Abbas & Siddique (2024). *Proc. IMechE Part C*, 238(13). https://doi.org/10.1177/09544062231224525
- JMC. *iHSV Integrated AC Servo Motor Manual*. https://www.jmc-motor.com/uploads/20240730/1cf02d411ba42d6a61bb4df55b64b835.pdf

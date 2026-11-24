# Bitácora

Registro de avances, decisiones y hallazgos. Agregar entradas arriba, con fecha.

| Fecha | Tipo | Entrada |
|---|---|---|
| 2026-11-24 | Documentación | El repositorio se organiza por secciones (plataforma, Paper 1, Paper 2) y se actualiza toda la información a firmware 1.3, app R5B Lab 0.4.1, BNO055 y geometría v3 |
| 2026-11-23 | Paper 1 | Sesiones S3 (segundo bloque del DOE) y S4 (carga de 100 g en A y D; brazos bloqueados con el utillaje y servos apagados). Análisis completo y borrador del manuscrito |
| 2026-11-23 | Hallazgo | Con brazos bloqueados y servos apagados la oscilación de 1–5 Hz desaparece: el modo viene del lazo del servo contra la inercia, no de la flexión de los eslabones |
| 2026-11-22 | Mantenimiento | Prisioneros nuevos sobre cara plana con traba roscas, chumaceras alineadas, cable de la IMU reacomodado. Los datos anteriores quedan como pre-mantenimiento |
| 2026-11-22 | Paper 1 | S1 (reposo, calibración con video 1,15–1,17 mm RMS, piloto de amplitudes 4/12/36 mm) y S2 (primer bloque del DOE). App 0.4.1 con la excitación P → P + Δ → P |
| 2026-11-22 | Hallazgo | La frecuencia natural baja con la amplitud en todas las posturas (en C: 3,6 → 2,3 → 1,8 Hz): ablandamiento |
| 2026-11-21 | Diseño | Reductor planetario 6:1 impreso (sol z12, planetas z24, corona z60, m 1,5) para bajar la relación de inercia; piezas de prueba de holgura |
| 2026-11-18 | Decisión | Tras el informe de avance, el docente recomienda dos papers: identificación dinámica (TecnoLógicas) y control inercial (Ingeniería e Investigación) |
| 2026-11-16 | Software | Firmware 1.4 (prealimentación por modelo) y app 0.4.0 (prealimentación y carga) listos y validados en el PC; aún no probados en el robot |
| 2026-11-16 | Decisión | Objetivos de Técnicas de Control reducidos a tres: EKF, leyes de control (amortiguamiento + prealimentación) y evaluación con/sin IMU y carga |
| 2026-11-15 | Experimento | Primer bloque pareado con/sin IMU (firmware 1.3, app 0.3.5, α = 0,3): a 200/2000 el contorno baja 35–38 % y el sobrepaso al parar 55–68 % |
| 2026-11-15 | Hallazgo | La repetibilidad del pick and place empeora por una deriva mecánica en los acoples; el cable de la IMU tenso perturba el giro. Se piden prisioneros y chumaceras nuevas |
| 2026-11-14 | Software | EKF validado offline contra el video; firmware 1.3 (EKF y corrección en la ESP32) y app 0.3.5. Ley elegida: inyección de amortiguamiento con lavado; LQR queda como comparación |
| 2026-11-13 | Decisión | Sintonía de los drivers congelada (Kp 15, ff 50 %, P02-11 300 ms, P01-04 6, P08-25 0). Subir P01-04 produce chillido e inestabilidad |
| 2026-11-13 | Experimento | Línea base "sin IMU" con video (figuras a 100/1000 y 200/2000, pick and place de 10 ciclos); cinemática nominal validada con video (2,1–2,3 mm RMS) |
| 2026-11-09 | Modelo | Etapa 2: modelo M(q) + servo resorte-amortiguador con retardo; identificación con los cuadrados de la etapa 1 |
| 2026-11-05 | IMU | Etapa 1 repetida con los eslabones nuevos (IMU dentro del distal derecho, a 33,5 mm del efector); análisis de golpes e inclinación |
| 2026-11-03 | Diseño | Eslabones v3 con rodamientos 6905-2RS en el efector y husillo del eje Z entre dos 608 |
| 2026-10-29 | Diseño | Eslabones v3 con cableado interno, torre Z (N20 + husillo T8) y garra (MG90S); IMU por la cadena derecha y actuadores por la izquierda |
| 2026-10-27 | IMU | Etapa 0 de la BNO055: ruido del giroscopio igual a la hoja de datos; 0,6 % de lecturas I²C corruptas con los servos energizados; pico de vibración en 13,2 Hz |
| 2026-10-22 | IMU | BNO055 verificada: dirección 0x29, oscilador interno, gz positivo en sentido antihorario (igual que los ángulos del robot) |
| 2026-10 | Diseño | Geometría v3: proximales 0,12 m, distales 0,18 m, separación de servos 0,15 m. Pines del eje 2 pasan a GPIO 15/16 |
| 2026-10 | Decisión | No se implementan los encoders AS5600: se trabaja solo con la IMU |
| 2026-10 | Decisión | Efector: torre Z con garra en lugar de ventosa |
| 2026-10-20 | Documentación | Se crea este repositorio por solicitud del docente |
| 2026-10-15 | Diseño | Geometría v2: proximales 0,15 m, distales 0,20 m, separación de servos 0,15 m |
| 2026-10-15 | Hallazgo | Con 90°/90° en el utillaje el robot queda en el modo de ensamblaje 1; modos seleccionables con `MODE n` |
| 2026-10-15 | Hallazgo | `jacobiano_analitico.m` usaba la rama inferior de la cinemática directa; se corrige pasando el punto P |
| 2026-10-15 | PAE | Se define el título de la PAE 2027-1 (control servo-visual con fusión cámara + IMU) |
| 2026-10-06 | IMU | Primera caracterización con una MPU6050 (GY-521), luego reemplazada por la BNO055 |
| 2026-10-02 | Hallazgo | La telemetría (θ1 = 88°, θ2 = 48,4°) mostró que el robot físico estaba en el modo 1 mientras el código usaba el modo 3 |
| 2026-10 | Software | Se empieza la app propia R5B Lab (tema claro, inicio, monitoreo, trayectorias) en reemplazo de la GUI de Tkinter |
| 2026-10 | Decisión | Cambio de IMU: de MPU6050 a BNO055 para el EKF |
| 2026-09-22 | Investigación | Estado del arte y análisis de 363 registros de Scopus |
| 2026-09 | Decisión | Técnicas de Control: EKF + IMU como realimentación activa, sin comparar con PD/CTC clásico |
| 2026-06 | Hardware | Servos JMC iHSV57-30-18-36: manuales y software de configuración (JMC 1.7.6) |

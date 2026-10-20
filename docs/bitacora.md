# Bitácora

Registro de avances, decisiones y hallazgos. Agregar entradas arriba, con fecha.

| Fecha | Tipo | Entrada |
|---|---|---|
| 2026-10-20 | Documentación | Se crea este repositorio por solicitud del docente: fabricación, hardware, control, software y arquitectura de software |
| 2026-10-17 | Decisión | Por ahora sin encoders externos: se empieza solo con la IMU; luego se integran dos AS5600 |
| 2026-10-15 | Diseño | Geometría v2: proximales 0,15 m, distales 0,20 m, separación de servos 0,15 m |
| 2026-10-15 | Hallazgo | Con 90°/90° en el utillaje el robot queda en el modo de ensamblaje 1; se trabaja en modos seleccionables (`MODE n`) |
| 2026-10-15 | Hallazgo | `jacobiano_analitico.m` usaba la rama inferior de la cinemática directa; se corrige pasando el punto P |
| 2026-10-15 | PAE | Se define el título de la PAE 2027-1 (control servo-visual con fusión cámara + IMU) |
| 2026-10-06 | IMU | Etapa 0: firmware y script de análisis para caracterizar la MPU6050 |
| 2026-10-02 | Hallazgo | La telemetría (θ1 = 88°, θ2 = 48,4°) mostró que el robot físico estaba en el modo 1 mientras el código usaba el modo 3 |
| 2026-10 | Software | Se empieza el diseño visual de la aplicación propia (tema claro, pantalla de inicio, monitoreo, trayectorias) |
| 2026-10 | Decisión | Cambio de IMU: de MPU6050 a BNO055 para el EKF |
| 2026-10 | Decisión | Efector final: ventosa + mini bomba de vacío |
| 2026-09-22 | Investigación | Estado del arte y análisis de 363 registros de Scopus |
| 2026-09 | Decisión | Técnicas de Control: EKF + IMU como realimentación activa, sin comparar con PD/CTC clásico |
| 2026-06 | Hardware | Servos JMC iHSV57-30-18-36: manuales y software de configuración (JMC 1.7.6) |

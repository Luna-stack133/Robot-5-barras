# IMU BNO055: etapas del trabajo

La IMU del robot es una **BNO055** dentro del eslabón distal derecho (ver [docs/02-hardware.md](../docs/02-hardware.md#imu-bno055)). El trabajo con ella se hizo por etapas; cada carpeta tiene su guía (`LEEME.md`).

| Etapa | Carpeta | Qué se hizo | Resultado principal |
|---|---|---|---|
| 0 | `etapa0_bno055/` | Conocer, calibrar y caracterizar la BNO055 en reposo (servos apagados y energizados) | Ruido de gz 0,10 °/s (0,0135 °/s/√Hz, igual a la hoja de datos); 0,6 % de lecturas I²C corruptas con los servos energizados (ya se filtran); valores iniciales de R y Q para el EKF |
| 1 | `etapa1/` | Comparar el giro medido con el que predice la cinemática en las corridas de la app; prueba de golpes; inclinación; calibración cinemática con la IMU | Oscilación de 2–4 Hz después de cada parada, dependiente de la postura; escala efectiva del giroscopio k = 1,031 |
| 2 | `etapa2_modelo_ekf/` | Modelo dinámico M(q) + servo, identificación, EKF y LQR offline, simulación del lazo con el firmware, métricas con video | Modelo que explica la vibración después de las paradas; EKF validado contra el video; ley α con lavado elegida para el firmware 1.3 |

La identificación definitiva (después del mantenimiento, con diseño factorial y validación fuera de muestra) es la del Paper 1: [`paper1-tecnologicas/`](../paper1-tecnologicas/README.md).

Los scripts leen las corridas que guarda la app R5B Lab en `Descargas\Resultados_BNO055\etapa1\`.

La primera caracterización con una MPU6050 (GY-521), de octubre de 2026, se retiró de esta carpeta porque ese sensor ya no se usa; queda en el historial de git.

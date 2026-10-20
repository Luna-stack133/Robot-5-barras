# Pendientes

## Para corregir en el código

- [ ] **Geometría desactualizada.** `kinematics_5bar.h` tiene la v1 (0,090 / 0,165 / 0,155 m). Actualizar a la v2 (0,15 / 0,20 / 0,15 m). La simulación (`main.m`) y los valores por defecto de la GUI usan 0,25 / 0,30 / 0,30 m.
- [ ] **Modo de ensamblaje del firmware.** El firmware de este repositorio solo usa el modo 3 (sgn = [+1, +1]), pero el utillaje a 90°/90° deja el robot en el modo 1. Con la geometría actual del firmware, la cinemática inversa del punto de cero da θ2 = 126,4° en vez de 90°, así que el primer `GOTO` después de `SETHOME` le pediría al eje 2 un salto grande. Subir la versión del firmware con modos seleccionables (`MODE n`) o cambiar el utillaje.
- [ ] **Límites articulares** a los topes mecánicos reales.
- [ ] **Medir los ángulos reales del utillaje** y ponerlos en `TH1_HOME_DEG / TH2_HOME_DEG`.
- [ ] `ternario.m` estaba vacío en la carpeta original; no se subió.

## Versiones que faltan en el repositorio

- [ ] Firmware y GUI con modos de ensamblaje seleccionables, geometría v2 y límites ampliados (si los tiene otro integrante, subirlos aquí).
- [ ] Bocetos o código de la nueva aplicación (GUI propia).

## Para documentar

- [ ] Fotos del robot, el cableado y el utillaje (`docs/img/`)
- [ ] Archivos CAD/STL y método de fabricación de cada pieza (`hardware/cad/`)
- [ ] Lista de materiales con costos
- [ ] Fuente de alimentación y protección contra regeneración
- [ ] Parámetros configurados en los servos (DIP / software JMC)
- [ ] Diseño del gripper (Roiman)
- [ ] Resultados de la etapa 0 de la IMU (JSON y gráficas)
- [ ] Videos de las pruebas de trayectorias

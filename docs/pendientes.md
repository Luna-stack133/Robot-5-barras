# Pendientes

## Experimentos

- [ ] **Paper 2:** repetir el bloque pareado con/sin IMU después del mantenimiento (figuras a 100/1000 y 200/2000, pick and place de 10 ciclos), con video. Antes, verificación corta en modo Sombra (golpes y secuencia de 18 puntos) para ver si el modelo sigue valiendo.
- [ ] Actualizar los parámetros del modelo en el firmware (`CTL MOD` en la 1.4, o `ekf_5b.h`) con los identificados en el Paper 1 después del mantenimiento.
- [ ] Probar en el robot la prealimentación por modelo (firmware 1.4), primero con ganancia 0,5 y a 100/1000.
- [ ] Pick and place con carga real (garra con objeto).
- [ ] Ensayos opcionales del Paper 1: golpe con servos apagados y brazos libres; brazos sujetos con servos activos; latencia del comando al movimiento (ver `paper1-tecnologicas/docs/ensayos_opcionales.md`).

## Mecánica

- [ ] Reductor planetario 6:1: elegir la holgura con las piezas de prueba, medir el eje libre y la rosca, y diseñar la carcasa definitiva.
- [ ] Control del eje Z (N20 con encoder, lazo de posición en la ESP32) y de la garra (MG90S) desde el firmware.
- [ ] Distales más rígidos y juntas con dos rodamientos (después de cerrar las campañas de los papers).
- [ ] Revisar la inclinación del montaje de la IMU (pitch ≈ −4,65°) y el origen del pico de 13,2 Hz.

## Para corregir en el código

- [ ] **Límites articulares** a los topes mecánicos reales (hoy [−170°, 350°] calculados para cubrir la zona útil).
- [ ] Integrar en la app la acción de la garra (hoy las acciones Tomar / Soltar de las secuencias solo se registran).
- [ ] Revisar `simulacion/matlab` con la geometría v3 (se actualizaron `main.m` y los puntos del triángulo).
- [ ] `ternario.m` estaba vacío en la carpeta original; no se subió.

## Para documentar

- [ ] Fotos del robot completo, el cableado, el utillaje y el efector (`docs/img/` y `paper1-tecnologicas/docs/fotos/`)
- [ ] Archivos CAD/STL de los eslabones v3, la torre Z, la garra y el utillaje (`hardware/cad/`)
- [ ] Lista de materiales con costos
- [ ] Fuente de alimentación y protección contra regeneración
- [ ] Diseño del gripper (Roiman)
- [ ] Videos de las pruebas de trayectorias (enlace a Drive o Zenodo)

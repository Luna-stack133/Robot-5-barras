# 01 · Fabricación

## Descripción del mecanismo

Robot paralelo tipo **SCARA paralelo de 3 GDL**. El plano X-Y es un mecanismo de cinco barras (configuración 5R): dos eslabones **proximales** unidos directamente a los ejes de los servos (O1 y O2), dos eslabones **distales** que se unen en el efector P, y la base fija entre los dos motores. El tercer grado de libertad es un **eje lineal Z** en el efector, con una garra en la punta.

```
            P (efector + torre Z + garra)
           /  \
       L2 /    \ L3        distales
         /      \
        A        B         codos
   L1  /          \  L4    proximales
      /            \
    O1 ─────D────── O2     ejes de los servos (base fija)
```

- **Montaje:** plano horizontal (la gravedad no actúa en el plano X-Y).
- **Transmisión:** directa (direct drive). Los proximales van montados sobre los ejes de los servos con un acople y prisioneros, sin reductor.
- **Origen del sistema de coordenadas:** O1 = (0, 0), O2 = (D, 0); el espacio de trabajo útil está en y > 0.

## Geometría

| Parámetro | Símbolo | v1 (primer prototipo) | v2 (oct. 2026) | **v3 (vigente, desde fines de oct. 2026)** |
|---|---|---|---|---|
| Proximal izquierdo | L1 | 0,090 m | 0,150 m | **0,120 m** |
| Distal izquierdo | L2 | 0,165 m | 0,200 m | **0,180 m** |
| Distal derecho | L3 | 0,165 m | 0,200 m | **0,180 m** |
| Proximal derecho | L4 | 0,090 m | 0,150 m | **0,120 m** |
| Separación entre ejes de los servos | D | 0,155 m | 0,150 m | **0,150 m** |

Las medidas son entre centros de articulación (CAD centro a centro: proximal 120,00 mm, distal 180,00 mm). La calibración cinemática con video del 22 nov 2026 confirmó la geometría nominal: 1,15–1,17 mm RMS en 18 puntos de reposo, con proximales ajustados de 119,8–119,9 mm.

**Masas medidas (balanza):** proximal 115 g, distal 170 g, efector con torre Z y garra 334 g. Inercia del rotor de cada servo: 0,3 kg·cm².

**Regla:** si cambian las medidas, se edita **solo** `firmware/robot_5bar_esp32/kinematics_5bar.h`; la app R5B Lab la recibe del firmware con `GEOM` al conectarse. En MATLAB se actualizan `p.L1…p.d` en `simulacion/matlab/main.m`. Al cambiar la geometría hay que revisar los límites articulares, el ángulo del utillaje, el modo de ensamblaje que resulta y repetir la calibración con video.

## Modos de ensamblaje

Para un mismo punto (x, y) hay cuatro soluciones de la cinemática inversa, según hacia dónde queden los "codos" A y B: θᵢ = φᵢ + sgnᵢ·βᵢ. La numeración es la misma en el firmware, la app y `ik5bar.m`.

| Modo | sgn | Codos |
|---|---|---|
| 1 | [+1, −1] | ambos hacia afuera (por defecto) |
| 2 | [−1, +1] | ambos hacia adentro |
| 3 | [+1, +1] | izquierdo afuera, derecho adentro |
| 4 | [−1, −1] | izquierdo adentro, derecho afuera |

El modo físico lo fija el utillaje de cero. Con la geometría v3 y el utillaje a 90°/90° el robot queda en el **modo 1**, con el efector en (75; 283,6) mm. `SETHOME` detecta el modo, y `MODE n` lo cambia con un movimiento articular verificado (sin abrir la cadena ni cruzar la singularidad paralela). Llegar al modo 2 casi nunca es seguro, porque los brazos pasarían por debajo de la base.

## Cero mecánico: utillaje de referencia

El robot no tiene sensores de posición absoluta ni finales de carrera. El cero se fija con un **utillaje** impreso en PLA que bloquea los dos proximales en θ1 = θ2 = 90°.

1. Acercar los brazos al utillaje con JOG articular (app → Monitoreo).
2. Encajar el utillaje.
3. **Fijar cero** (`SETHOME`): el firmware fija el contador de pasos en esos ángulos (no mueve nada).
4. Retirar el utillaje.

El mismo utillaje sirve para bloquear los brazos en las pruebas de diagnóstico con los servos apagados (Paper 1, prueba D2).

## Eslabones v3 con cableado interno (29 oct – 3 nov 2026)

- **Proximales:** la geometría de los archivos Slave1v3 / Slave3v4, cortados por la cara de abajo para un canal de cable de 7 × 5 mm con tapa (3 × M2). El cable entra por la tapa a 47,5 mm del eje del motor, por fuera del acople.
- **Distales:** contorno de Slave2v3 (codo R25, 12 mm de espesor). Canal por debajo con tapa (D3 izquierdo, D4 derecho). El distal derecho lleva el bolsillo de la IMU; el izquierdo, la manga de la torre Z.
- **Codos con eje macizo:** el cable sale del proximal por arriba, cruza ~95 mm por el aire por el lado de adentro del ángulo y entra al distal por abajo.
- **Separación vertical en los codos:** 15 mm. Capas: proximal derecho 0–12 mm, proximal izquierdo 13–25 mm, distal derecho 27–39 mm, distal izquierdo 40–52 mm.
- **Reparto de cables:** actuadores del efector (N20 con encoder y servo de la garra) por la cadena izquierda; IMU por la cadena derecha.
- **Material:** ABS; dedos de la garra en TPU; utillaje en PLA.

## Efector: torre Z y garra

- **Rodamientos del efector:** 6905-2RS (25 × 42 × 9 mm). El 6805-2RS original casi no se consigue en Colombia.
- **Eje Z:** motorreductor N20 con encoder (placa CH-N20-3) → engranajes módulo 1, z16/z32 (2:1, 24 mm entre ejes) → husillo T8 de 100 mm y 4 entradas (8 mm por vuelta) → tuerca de brida en la tapa del ariete → tubo cuadrado de aluminio de 1/2". Carrera de 50 mm. El husillo queda sujeto entre dos rodamientos 608 (placa de la torre y tapa Z6).
- **Garra:** accionada por un servo MG90S, dedos en TPU.
- **Control:** la garra y el eje Z todavía **no** se manejan desde la ESP32. En las corridas de pick and place la garra va vacía.
- La ventosa con mini bomba de vacío que se había elegido al inicio se descartó a favor de la garra.

## Montaje de la IMU

La BNO055 va **dentro del eslabón distal derecho**, en un bolsillo abierto por abajo y sostenido por la tapa D4: placa atravesada (21 mm × 13 mm), pines cortados hacia el codo, chip hacia arriba, con su centro a **33,5 mm del eje del efector**. El cable va por el canal interno de la cadena derecha, con holgura en el codo y sujeto en los dos eslabones (un cable tenso endurece el giro y perturba las mediciones).

## Mantenimiento del 22 nov 2026

- Prisioneros nuevos en los acoples, sobre la cara plana del eje, con traba roscas y marcas de testigo pintadas.
- Chumaceras alineadas (el eje gira libre antes de apretar).
- Cable de la IMU reacomodado para que no se interponga en las trayectorias.

Todos los datos tomados antes de esa fecha quedan como *pre-mantenimiento*. Después del mantenimiento se repitieron la calibración con video y los golpes de referencia (V1: 2,33–2,34 Hz, frente a 2,22–2,32 Hz antes).

## Hallazgos mecánicos

- **Flexión de los distales:** el efector de 334 g en voladizo, las juntas con un solo rodamiento y la poca altura de los eslabones dejan inclinar el efector. Se ve en la inclinación que mide la IMU según la postura (2–2,8°).
- **Relación de inercia carga/rotor de 105–360** según la postura: muy por encima de lo que estos servos manejan bien (< 10–30). Es la causa de fondo de la oscilación de 1,6–3,7 Hz después de cada parada (ver [03-control.md](03-control.md)).

## Mejoras mecánicas

| Mejora | Motivo | Estado |
|---|---|---|
| Reductor planetario 6:1 impreso (sol z12, planetas z24, corona z60, módulo 1,5; rodamientos 608) | Bajar la relación de inercia a ~6–12 y atacar la oscilación | En diseño; piezas de prueba de holgura entregadas (21 nov) |
| Distales con sección de caja o en I, 20–25 mm de alto | Rigidez a flexión | Planeado (después de las campañas de los papers) |
| Juntas con dos rodamientos separados 15–20 mm | Quitar la inclinación de los ejes | Planeado |
| Control del eje Z y de la garra desde la ESP32 | Pick and place con carga | Planeado |
| Estructura para la cámara | Pick and place con visión (PAE) | Planeado |
| Cero con sensor (sin finales de carrera) | Reemplazar el utillaje manual | En evaluación |
| ~~Encoders magnéticos AS5600~~ | — | Descartado: se trabaja solo con la IMU |

**Regla:** cualquier cambio mecánico obliga a repetir la línea base (golpes, calibración con video y trayectorias), porque cambian el modelo y las masas.

## Por documentar

- [ ] Parámetros de impresión de cada pieza (relleno, altura de capa, perímetros)
- [ ] Archivos CAD/STL de eslabones v3, efector, torre Z, garra y utillaje (`hardware/cad/`)
- [ ] Lista de materiales (BOM) con costos
- [ ] Fotos del montaje (`docs/img/`)

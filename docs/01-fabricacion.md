# 01 · Fabricación

## Descripción del mecanismo

Robot paralelo plano de cinco barras (configuración 5R): dos eslabones **proximales** unidos directamente a los ejes de los servos (O1 y O2), dos eslabones **distales** que se unen en el efector final P, y la base fija entre los dos motores. Tiene 2 GDL (posición x, y del efector en el plano).

```
            P (efector)
           /  \
       L2 /    \ L3        distales
         /      \
        A        B         codos
   L1  /          \  L4    proximales
      /            \
    O1 ─────D────── O2     ejes de los servos (base fija)
```

- **Montaje:** plano horizontal (el robot trabaja sobre una mesa; la gravedad no actúa en el plano de movimiento).
- **Transmisión:** directa (direct drive). Los eslabones proximales van montados sobre los ejes de los servos, sin reductor.
- **Origen del sistema de coordenadas:** O1 = (0, 0), O2 = (D, 0); el espacio de trabajo útil está en y > 0.

## Geometría

| Parámetro | Símbolo | v1 (primer prototipo) | v2 (vigente, oct. 2026) |
|---|---|---|---|
| Proximal izquierdo | L1 | 0,090 m | 0,150 m |
| Distal izquierdo | L2 | 0,165 m | 0,200 m |
| Distal derecho | L3 | 0,165 m | 0,200 m |
| Proximal derecho | L4 | 0,090 m | 0,150 m |
| Separación entre ejes de los servos | D | 0,155 m | 0,150 m |

Las medidas son entre centros de articulación.

> ⚠️ **La geometría no está igual en todos los archivos de este repositorio.** El firmware (`kinematics_5bar.h`) todavía tiene la v1; la GUI y la simulación de MATLAB usan valores de ejemplo (0,25 / 0,30 / 0,30 m). La GUI se corrige sola al conectarse porque le pide la geometría al firmware (comando `GEOM`), pero la simulación no. Ver [pendientes.md](pendientes.md).

**Regla:** si cambian las medidas, se edita **solo** `firmware/robot_5bar_esp32/kinematics_5bar.h` (y `p.L1…p.d` en `simulacion/matlab/main.m`). Al cambiar la geometría hay que revisar de nuevo los límites articulares, el ángulo del utillaje y el modo de ensamblaje que resulta.

## Modos de ensamblaje

Para un mismo punto (x, y) hay cuatro soluciones de la cinemática inversa, según hacia dónde queden los "codos" A y B: θᵢ = φᵢ + sgnᵢ·βᵢ.

| Modo | sgn | Codos |
|---|---|---|
| 1 | [+1, −1] | ambos hacia afuera |
| 2 | [−1, +1] | ambos hacia adentro |
| 3 | [+1, +1] | izquierdo afuera, derecho adentro |
| 4 | [−1, −1] | izquierdo adentro, derecho afuera |

El modo en que queda el robot físico lo fija el utillaje de cero. Con la geometría v2 y el utillaje a 90°/90°, el robot queda en el **modo 1**, con el efector en (0,075; 0,335) m. El firmware de este repositorio está escrito para el modo 3 (ver [pendientes.md](pendientes.md)).

## Cero mecánico: utillaje de referencia

Por ahora el robot no tiene sensores de posición absoluta ni finales de carrera. El cero se fija con un **utillaje**: una pieza que bloquea los dos brazos proximales en una pose única y conocida (θ1 = θ2 = 90° por defecto).

Procedimiento:
1. Acercar los brazos al utillaje (comando `JOG` desde la GUI).
2. Encajar el utillaje.
3. `SETHOME` → el firmware fija el contador de pasos en esos ángulos (no mueve nada).
4. Retirar el utillaje.

*Pendiente: subir el modelo del utillaje a `hardware/cad/` y medir los ángulos reales que deja.*

## Efector final

- Seleccionado: **ventosa + mini bomba de vacío** (adecuado para el montaje horizontal y para pick and place de objetos planos).
- El diseño del gripper adaptativo lo lleva Roiman. *Pendiente: documentar diseño, piezas y montaje.*

## Montaje de la IMU

La IMU va atornillada y plana sobre el eslabón distal, cerca del efector, con el eje X de la placa a lo largo del eslabón. Cable corto y flexible, SDA y SCL trenzados cada uno con un GND, con holgura en las articulaciones y lejos de los cables de potencia de los servos.

## Mejoras mecánicas planeadas

| Mejora | Motivo | Estado |
|---|---|---|
| Soporte para encoders magnéticos AS5600 | Medir el ángulo real de cada proximal. Imán en la cara superior del proximal, centrado en el eje del motor; sensor en un soporte encima | Planeado |
| Reductores en los servos | Más par, aceleración y velocidad | En evaluación |
| Estructura para la cámara | Pick and place con visión (PAE) | Planeado |
| Cero con sensor (sin finales de carrera) | Reemplazar el utillaje manual | En evaluación |

## Por documentar

- [ ] Material y método de fabricación de cada pieza (impresión 3D, corte, mecanizado), con parámetros
- [ ] Planos o archivos CAD/STL de eslabones, base, acoples al eje y utillaje (`hardware/cad/`)
- [ ] Rodamientos, pasadores y tornillería de las articulaciones
- [ ] Lista de materiales (BOM) con costos
- [ ] Fotos del montaje (`docs/img/`)

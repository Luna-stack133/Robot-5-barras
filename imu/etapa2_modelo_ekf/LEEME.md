# Etapa 2 — Modelo dinámico del robot (primera parte)

## Qué hay en esta carpeta

| Archivo | Qué hace |
|---|---|
| `modelo5b.py` | El modelo: cinemática, inercia según la postura M(q) con las masas medidas, el servo como lazo de posición y la simulación. Lo usará también el EKF |
| `identificar.py` | Ajusta los parámetros del servo con los cuadrados de la etapa 1 y compara el modelo con lo medido |

## El modelo

Para cada motor i:

  M(q)·q̈ + C(q, q̇)·q̇ = K_i·(q_c(t − τ) − q) + B_i·(ff·q̇_c(t − τ) − q̇)

- **q_c** es el ángulo que manda la ESP32 (los pulsos) y **q** el ángulo real del motor. La diferencia entre los dos es lo que los pulsos no ven.
- **M(q)** es la inercia que siente cada motor en cada postura. Sale de las masas medidas: proximal 115 g, distal 170 g y efector (torre Z + garra) 334 g, tomado como masa puntual.
- **K y B** son la rigidez y el amortiguamiento del lazo de posición de cada servo. **ff** es el feedforward de velocidad y **τ** el retardo.
- **La IMU mide** gz = 1.031 · (giro del distal derecho).

## Uso

```
python identificar.py
```

Lee las corridas de `Descargas\Resultados_BNO055\etapa1`. De cada par de cuadrados usa uno para ajustar y el otro para validar, y deja también para validar las secuencias de calibración y la corrida a 275/1575. Los resultados quedan en `Descargas\Resultados_BNO055\etapa2\`:
- `modelo_identificado.json`: los parámetros;
- `modelo_<corrida>.png`: medido frente a la cinemática rígida y al modelo dinámico, y el error de cada uno.

Con `--comun` usa el mismo K y B para los dos motores. Por defecto usa uno por motor.

## Resultados (9 de noviembre de 2026)

| Parámetro | Motor 1 | Motor 2 |
|---|---|---|
| Rigidez K [N·m/rad] | 1.58 | 1.36 |
| Amortiguamiento B [N·m·s/rad] | 0.017 | 0.050 |
| Feedforward ff | 0.52 (común) | |
| Retardo τ | 33 ms (común) | |

Error del giro del distal derecho en las corridas de **validación** (no usadas en el ajuste):

| Corrida | Cinemática rígida | Modelo dinámico |
|---|---|---|
| Cuadrado 100/200 | 5.0 °/s | 4.6 °/s |
| Cuadrado 100/1000 | 9.1 °/s | 6.1 °/s |
| Cuadrado 100/2000 | 9.2 °/s | 5.7 °/s |
| Cuadrado 200/2000 | 9.7 °/s | **4.8 °/s** |
| Cuadrado 275/1575 | 16.3 °/s | 11.1 °/s |
| Secuencias 50/200 | 3.1–3.2 °/s | 2.8–3.0 °/s |
| Cuadrado lento 20/100 | 2.3 °/s | 2.3 °/s |

**El modelo explica la vibración después de las paradas:** en las corridas rápidas reproduce la oscilación en frecuencia, amplitud y fase, y el error baja a la mitad. En las corridas de validación el error es igual que en las de ajuste, así que el modelo no está memorizando los datos.

**Lo que el modelo no explica:**
1. **Una oscilación sostenida a baja velocidad** (cuadrado a 20 mm/s): ~2 Hz y ±4 °/s, con el comando completamente suave, y algún "salto" suelto de hasta 16 °/s. Tiene la forma de un **pegue-deslice (stick-slip)**: la fricción de las articulaciones contra un servo blando con una integral lenta. Un modelo lineal no puede producirla; agregar una fricción simple tampoco la reproduce.
2. **La rigidez que depende de la amplitud** (ya vista en los golpes). Los K identificados (1.4–1.6 N·m/rad) corresponden a las amplitudes de los cuadrados. Con golpes pequeños daba ~2.75.

Los parámetros son **efectivos**: absorben en parte esos efectos no lineales. Por ejemplo, K/B no da el Kp = 20 del driver.

## Consecuencia para el EKF

Lo que el modelo no predice (la fricción, el pegue-deslice, la rigidez no lineal) se trata como una **perturbación de par desconocida en cada motor**, que el EKF estima con la IMU. Es justamente donde la IMU aporta: el modelo predice lo que sí sabe y la medición corrige lo demás.

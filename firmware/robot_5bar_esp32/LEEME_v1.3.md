# Firmware 1.3: EKF y corrección con IMU

Es el firmware 1.x que ya usan (misma generación de pulsos con FastAccelStepper, mismo streaming de 4 ms,
mismos comandos) con tres agregados:

1. **EKF en la ESP32.** Corre a 100 Hz en la misma tarea que lee la BNO055 (núcleo 0). Estima los ángulos, las
   velocidades y los pares de perturbación de los dos brazos, más el sesgo del giróscopo. Usa el modelo
   identificado el 13 nov, con la sintonía congelada del driver.
2. **Ley de corrección.** Calcula Δq y el lazo de trayectoria la **suma a los pulsos** cada 4 ms. Por defecto
   se usa la inyección de amortiguamiento:

   Δq = −K⁻¹·D·zona(ω̂ − q̇_plan), con D = 2α√(K·M)

   Lleva filtro de lavado (1 s), recorte de ±1° y un límite de cambio de 0.3° por muestra. También está el
   LQR con retardo, pero es experimental (ver abajo).
3. **Modos.** El firmware **arranca Apagada** y entonces se comporta igual que el 1.x.
   - **Sombra**: el EKF calcula y registra, pero no aplica la corrección.
   - **Activa**: la corrección se suma a los pulsos.

   Se elige desde la app 0.3.5 (pantalla *Control y estimación*) o con `CTL MODO 0|1|2` desde el Monitor Serie.

## Archivos (todos en la misma carpeta)

| Archivo | Qué es |
|---|---|
| `robot_5bar_esp32.ino` | Programa principal. Agrega el registro del plan, la suma de Δq, `updateIdle()` y los comandos `CTL`. |
| `kinematics_5bar.h` | Sin cambios. |
| `imu_bno055.h` | Corre el EKF después de cada lectura. La línea `I` lleva 10 campos más. La pila de la tarea sube a 12 KB. |
| `control_imu.h` | Puente entre la tarea de la IMU y `loop()`: plan, modos, seguridad y comandos. |
| `ekf_5b.h` | EKF y ley de corrección. No depende de Arduino; se probó en el PC. |
| `lqr_tabla.h` | Ganancias LQR por postura (generado por `gen_lqr_tabla.py`). |

Se compila igual que antes: Arduino IDE, placa ESP32-S3 y la librería FastAccelStepper.

## Seguridad

- La corrección se recorta a ±1° por motor y su cambio a 0.3° cada 10 ms.
- El filtro de lavado impide que sostenga un valor constante. Por eso un sesgo del acelerómetro no deja error
  de posición en reposo.
- Solo actúa en movimiento cartesiano y en reposo con el cero fijado. No actúa durante JOG, cambio de modo de
  ensamblaje, HALTED ni sin cero. Después de STOP, JOG o SETHOME el EKF se reinicia.
- El firmware pasa solo a **Sombra** y envía `EVT CTL_OFF <motivo>` en cualquiera de estos casos (la app lo
  muestra en rojo):
  - la IMU deja de leer durante 50 ms (`SIN_LECTURA`);
  - entrega 5 lecturas inválidas seguidas (`IMU_INVALIDA`);
  - el giróscopo difiere del EKF más de 30°/s durante 50 ms (`INNOVACION`);
  - el EKF da un número inválido (`EKF`).

  Si la tarea de la IMU se detuviera, `loop()` deja de sumar la corrección a los 60 ms.
- En cualquier momento se puede volver a **Apagada**: la corrección baja a cero en rampa, en menos de 40 ms.

## Comandos nuevos

```
CTL                      -> CTL modo=0 ley=ALFA alfa=0.300 zona=1.00 dmax=1.000 lavado=1.00 ekf=0 t_ekf=812/1350
CTL MODO 0|1|2           apagada / sombra / activa
CTL LEY ALFA|LQR
CTL ALFA 0.3             amortiguamiento agregado (fracción del crítico)
CTL ZONA 1.0             zona muerta del error de velocidad (°/s)
CTL DMAX 1.0             recorte de la corrección (°)
CTL LAVADO 1.0           constante del filtro de lavado (s, 0 = sin filtro)
CTL GRAV cx cy p1 p2     ajuste de gravedad del acelerómetro (si se recalibra)
CTL RESET                reinicia el EKF
```

`t_ekf` es lo que tarda el EKF más la ley en la ESP32, en microsegundos: el último y el máximo. Debe quedar
muy por debajo de 10 000.

La línea `I` agrega: `q1h q2h` (milésimas de grado), `w1h w2h` (centésimas de °/s), `d1 d2` (décimas de mN·m),
`dl1 dl2` (corrección de la ley, milésimas de grado), `inov` (centésimas de °/s) y `estado`. En `estado`, los
bits 0-1 son el modo, el bit 2 indica EKF válido y el bit 3 que se está aplicando. La app 0.3.4 sigue
funcionando con esta línea (ignora los campos nuevos); la 0.3.5 los guarda.

## Cómo se validó (en el PC, antes de tocar el robot)

- **EKF contra Python.** El EKF de `ekf_5b.h` se comparó con el de Python sobre 9 corridas grabadas. Contra
  el video, el error queda en 3.49 mm (Python: 3.19 mm; el comando: 8.02 mm).
- **Lazo cerrado con el firmware real.** Se ejecutó el firmware completo (`robot_5bar_esp32.ino` y sus
  archivos, sin modificar) contra una planta con el modelo identificado. Se simularon los tiempos de la ESP32:
  lectura I2C de 5 ms, cálculo de 2 ms y streaming de 4 ms. Está en `prueba_pc/lazo`.

  | Prueba (simulación) | Desviación máx. al llegar | ±1 mm | ±0.5 mm | Error en reposo |
  |---|---|---|---|---|
  | P&P 10 ciclos, apagada | 3.59 mm | 0.42 s | 0.70 s | 0.03 mm |
  | P&P 10 ciclos, activa α 0.3 | 2.59 mm | 0.16 s | 0.29 s | 0.02 mm |
  | Cuadrado 200/2000, apagada | contorno RMS 5.84 mm | 0.83 s | 1.10 s | 0.07 mm |
  | Cuadrado 200/2000, activa α 0.3 | contorno RMS 4.33 mm | 0.31 s | 0.36 s | 0.02 mm |

  - El modo Sombra produjo exactamente los mismos pulsos que Apagada.
  - `EVT DONE` llega a la misma hora (+8 ms por ciclo), así que los tiempos de ciclo son comparables.
  - Con un sesgo de 0.1 m/s² en el acelerómetro, el error en reposo queda en 0.03 mm con el lavado y en
    0.69 mm sin él.
  - Al cortar la IMU 0.5 s, la corrección bajó a 0 en 30 ms y llegó `EVT CTL_OFF SIN_LECTURA`.

## Sobre el LQR (experimental)

Sin sesgos, el LQR es un poco mejor que α en simulación. Pero realimenta la **posición** estimada, y el EKF
solo ve directamente el distal derecho, así que una de las dos direcciones del movimiento es poco observable.

En las corridas reales del 13 nov, la posición estimada en reposo queda hasta 0.65° corrida respecto al plan.
Con eso, el LQR mantendría Δq cerca de 0.8° en reposo, unos 1–2 mm de error. Con el lavado tampoco se arregla:
en la reproducción de las corridas se satura.

Por eso la ley del trabajo es **α con lavado**. El LQR queda para una comparación y como resultado del paper
(por qué la realimentación de velocidad es la adecuada con este sensado).

# Etapa 1 (repetición con los eslabones nuevos) — La IMU frente a la cinemática

Robot paralelo de 5 barras · Técnicas de Control · versión del 5 de noviembre de 2026

## Por qué se repite

Los eslabones nuevos (con eje Z y garra) pesan más y la IMU quedó **dentro del eslabón distal derecho**, con el chip hacia arriba, a **33.5 mm** del eje del efector. Con más masa, la resonancia de ~4 Hz y el retardo cambian, así que se repite una parte de la etapa 1 con la configuración final.

## Qué hay en esta carpeta

| Archivo | Qué hace |
|---|---|
| `etapa1_analizar.py` | Analiza las corridas y las grabaciones de golpes que guarda R5B Lab |
| `calibrar_cinematica.py` | Calibración cinemática con la IMU (sección 6) |

Novedades del script respecto a la versión del 28 de octubre:

- **Prueba de golpes:** las grabaciones de **Monitoreo → Grabar IMU** se analizan aparte. Por cada golpe calcula la frecuencia natural, el amortiguamiento ζ y el asentamiento. También mide la gravedad que se cuela en ax/ay con el robot quieto.
- **Inclinación:** con 2 posturas o más, separa la inclinación en dos partes:
  - **una fija**, que viene de la IMU dentro de su bolsillo;
  - **una que gira con el eslabón**, que viene del plano del robot inclinado.

  El resultado queda en `resumen_inclinacion.csv`.
- **Lecturas dañadas:** las lecturas que dañó el ruido del I²C se reemplazan por la anterior, y el número de reemplazos queda en los resultados.
- **Rama de la cinemática:** la primera muestra toma la rama de la posición inicial de la telemetría, así ya no se confunde de rama.
- **Asentamiento en las paradas:** se mide en una ventana de 1.5 s, que termina antes si el eslabón vuelve a moverse. También se cuentan las paradas que no alcanzaron a asentarse.
- **`--rehacer`:** vuelve a analizar todo y rehace los resúmenes.

## 1. Preparación

1. **Firmware:** `robot_5bar_esp32` con `imu_bno055.h` (sin cambios). No usen *Erase All Flash*, porque borra la calibración de la IMU.
2. **App:** R5B Lab **0.3.1**. Al conectar, en Monitoreo aparece la tarjeta **"IMU · grabar"** con la lectura en vivo (gz, |a|, roll, pitch y calibración). Si dice "esperando datos", el firmware no es el de la IMU o el I²C no responde.
3. **Nivel:** pongan un nivel de burbuja sobre la base, en dos direcciones, y nivelen la mesa si hace falta.
4. Fijen el cero con el utillaje, como siempre.

## 2. Prueba de golpes (primero)

1. Lleven el robot a una postura con **Trayectorias → Ir a un punto**.
2. En **Monitoreo → IMU · grabar**, elijan **20 s** y pulsen **Grabar IMU**.
3. Dejen el brazo quieto unos **3 s**. Luego den **golpecitos suaves al efector**, en el plano del robot y siempre en la misma dirección, separados unos **2 s** (5 a 7 golpes).
4. La grabación se guarda sola como una corrida más, con el nombre "Golpes (x, y) mm".

Repítanlo en **4 posturas** en las que el eslabón distal derecho apunte en direcciones bien distintas. Con la geometría actual (modo 1) sirven estas:

| Postura | x, y [mm] | Ángulo del distal derecho |
|---|---|---|
| A | 75, 280 | ~114° |
| B | 170, 220 | ~118° |
| C | 75, 200 | ~145° |
| D | 0, 150 | ~169° |

Si la garra o el eje Z no dejan llegar a alguna, usen otra cercana. Lo importante es que el ángulo del distal cambie al menos 20°, mejor 50° o más.

## 3. Corridas de figuras

En **Trayectorias**, el cuadrado (centro 75, 190; tamaño 100 mm), **dos veces cada fila**:

| # | Vmax [mm/s] | Amax [mm/s²] |
|---|---|---|
| 1 | 100 | 200 |
| 2 | 100 | 1000 |
| 3 | 200 | 2000 |
| 4 | 20 | 100 (lenta, para la ganancia por lado) |

Durante las corridas nadie toca la mesa. Cada corrida se graba sola, hasta 2.5 s después de terminar.

Además, **midan con calibrador** L1, L2, L3, L4 y D reales, de centro a centro de los ejes.

## 4. Análisis

Desde la carpeta del script:

```
python etapa1_analizar.py --todas --r_mm 33.5
```

- `--todas` analiza lo que aún no tiene resultados. `--todas --rehacer` vuelve a analizar todo.
- En las figuras, el eslabón se detecta solo; debe salir **der**. En los golpes se usa **der** por defecto.
- Si el script detecta mal el eslabón, indíquenlo con `--eslabon der`.

Quedan:

- **En cada corrida de figura:** `resultados.csv`, `gz_medido_vs_predicho.png`, `oscilacion_paradas.png` y `aceleracion_y_rumbo.png`.
- **En cada grabación de golpes:** `resultados.csv` y `golpes.png`.
- **En `Descargas\Resultados_BNO055\etapa1\`:**
  - `resumen_etapa1.csv`: una fila por corrida de figura;
  - `resumen_golpes.csv`: una fila por postura;
  - `resumen_inclinacion.csv`: la inclinación separada en parte fija y parte del plano.

## 5. Cómo leer los golpes

El script mide cada golpe con el **decremento logarítmico**: compara la amplitud y el tiempo entre extremos sucesivos (medios ciclos) de la oscilación libre. Empieza en el rebote, que es el extremo más grande, y descarta el primer medio ciclo después de él, porque ahí el dedo todavía influye.

| Resultado | Qué indica |
|---|---|
| fn y ζ (medianas) | El modo dominante en esa postura |
| fn y ζ por amplitud (grande ≥ 20 °/s, media 8–20, pequeña < 8) | Si cambian con la amplitud, el "resorte" del servo no es lineal |
| Asentamiento | Tiempo desde el rebote hasta quedar dentro de 3 σ del ruido |
| Inclinación fija | La IMU inclinada en su bolsillo: se resta como constante en el EKF |
| Inclinación del plano | El robot o la mesa desnivelados: depende del ángulo del eslabón |

En cada carpeta de golpes queda además `medios_ciclos.csv`, con la amplitud, fn y ζ de cada medio ciclo, y en `golpes.png` una gráfica de fn y ζ frente a la amplitud.

fn, ζ y el retardo son los parámetros del modo de 2.º orden del modelo en espacio de estados de la etapa 2.

## 6. Calibración cinemática con la IMU

Sirve para encontrar por qué la ganancia sale ~1.05 en lugar de 1.00, sin desmontar el robot.

**Idea.** En cada movimiento de un punto de reposo A a otro B se comparan:
- el giro **medido** del distal derecho: la integral de gz entre A y B, con el sesgo restado;
- el giro **predicho** por la cinemática.

Entre dos reposos la vibración y el retardo no cuentan: solo la geometría. Por mínimos cuadrados (Levenberg-Marquardt) se ajustan:
- los errores del cero de cada motor (δ1, δ2);
- las longitudes L1–L4;
- la escala del giroscopio (k).

D queda fijo en 150 mm. Los valores de diseño se usan como información previa (±2° y ±2 mm), así que un parámetro que los datos no logran determinar se queda cerca del diseño y el script lo marca como "no identificable".

**Corrida de calibración (R5B Lab 0.3.2):**
1. Fijen el cero con el utillaje.
2. **Trayectorias → Secuencia → Puntos de calibración.** Se cargan unos 18 puntos repartidos por el espacio de trabajo, con pausa de 2 s en cada uno, a 50 mm/s y 200 mm/s².
3. **Antes de ejecutar,** revisen en el mapa que la garra y el eje Z no choquen en ningún punto. Si algún punto es peligroso, quítenlo de la tabla.
4. Ejecuten la secuencia **dos veces**: una para ajustar y otra para validar. Durante la corrida nadie toca la mesa.
5. Después, devuelvan la velocidad y la aceleración a sus valores habituales.

**Análisis:**

```
python calibrar_cinematica.py --solo-pausa
```

Sin `--solo-pausa` usa también los "Ir a…" y los cuadrados que tengan reposos. Con `--libres d1_deg,d2_deg,L3,k` se ajustan solo esos parámetros.

En la terminal y en `etapa1\calibracion\` quedan:
- `calibracion_parametros.csv`: diseño, valor calibrado, incertidumbre y si es identificable;
- `calibracion_movimientos.csv`: cada movimiento, con su giro medido y el predicho antes y después;
- `geometria_calibrada.json`: para el modelo y el EKF de la etapa 2;
- `calibracion.png`: giro medido frente a predicho y el error de cada movimiento.

Con datos simulados (2 corridas de 18 puntos), el script recupera los errores del cero con ±0.5°, L3 con ±1 mm y la escala del giroscopio con ±1 %, y distingue un error de escala de uno de geometría.

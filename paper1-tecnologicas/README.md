# Paper 1 — Identificación dinámica del mecanismo X-Y (TecnoLógicas)

**Experimental Identification of Configuration-Dependent Dynamics and Actuation Constraints in a Low-Cost Parallel-SCARA Robot**

Datos, scripts y figuras del Paper 1. El análisis corre de principio a fin con un solo comando y reproduce los
números del manuscrito: parámetros identificados, ANOVA, Wilcoxon y figuras 1–6.

*English summary: raw logs, video-tracking files, the run table (`data/runs.csv`), the exclusion log and all analysis
scripts for the paper. `python scripts/run_all.py` regenerates every number, table and figure from the raw data (~15 min).*

## Estructura

```
paper1-tecnologicas/
├── README.md              ← este archivo
├── requirements.txt       ← versiones de las librerías con que se probó
├── data/
│   ├── runs.csv           ← una fila por corrida (132): sesión, pase, orden, postura, nivel, fecha y hora, archivos, exclusión
│   ├── exclusions.csv     ← log de exclusiones (corrida completa o solo métricas de video) con motivo y acción
│   └── raw/
│       ├── S1 … S4/Lecturas/<corrida>/   ← imu.csv, telemetria.csv, info.json de cada corrida (app R5B Lab 0.4.1)
│       ├── video_tracking/               ← posición del marcador cuadro a cuadro (un CSV por video) y registro.txt
│       ├── frames/                       ← cuadro de video usado en la Fig. 2
│       └── bitacora.xlsx                 ← bitácora de laboratorio (original)
├── adquisicion/           ← firmware 1.3 (ESP32-S3) y app R5B Lab 0.4.1 con que se tomaron los datos
├── scripts/               ← análisis (ver tabla abajo); lib/ = modelo del robot y reconstrucción de video;
│                             video_tracking/ = seguimiento del marcador en los .mp4
├── results/               ← salidas numéricas (CSV/JSON) que generan los scripts
├── figures/               ← figuras 1–6 (PNG 300 dpi + PDF vectorial)
└── docs/
    ├── notas/             ← plan de pruebas y revisiones de las sesiones (registro histórico)
    ├── fotos/             ← fotografía del robot completo y del montaje
    └── ensayos_opcionales.md
```

Los videos originales (.mp4, ~60 fps, 1080 × 1920) no están en el repositorio por su tamaño. Se guardan en
[enlace a Drive/Zenodo — completar]. `data/raw/video_tracking/` tiene todo lo que el análisis usa de ellos.

## Cómo correrlo

Requisitos: Python 3.11 o posterior (probado con 3.13) y las librerías de `requirements.txt`. El seguimiento del marcador
(`scripts/video_tracking/`, solo si se quiere rehacer desde los videos) necesita además `opencv-python`.

```bash
cd paper1-tecnologicas
python -m venv .venv
.venv\Scripts\activate          # Windows  (Linux/macOS: source .venv/bin/activate)
pip install -r requirements.txt
python scripts/run_all.py       # todo: ~20 min. Para retomar: python scripts/run_all.py --desde 7
```

Todas las rutas son relativas a la carpeta del paper (`scripts/rutas.py`), así que funciona desde cualquier lugar.

| Paso | Script | Qué hace | Salida |
|---|---|---|---|
| 0 | `video_tracking/` | (Previo, en el PC con los .mp4) seguimiento del marcador en cada video | `data/raw/video_tracking/*.csv` |
| 1–2 | `calibracion_video.py` | Calibración cinemática con video: 18 puntos de reposo por secuencia; error con la geometría nominal y con ceros/longitudes ajustados | `results/calib/secuencia1/`, `secuencia2/` |
| 3 | `analisis_imu.py` | Ajuste de oscilación amortiguada en cada llegada a P (giro z); mediana por corrida; R² ≥ 0.85 | `imu_llegadas.csv`, `imu_corridas.csv` |
| 4 | `identificacion.py` | Identificación LM con ff libre (solo para el análisis de identificabilidad) | `identificacion.json` |
| 5 | `identificacion.py` con `FF_FIJO=0.5` | Identificación reportada: K₁, K₂, B₁, B₂, τ; DE por jacobiano y jackknife; ajuste en entrenamiento y prueba; modelo de inercia constante | `identificacion_ff0.5.json` |
| 6 | `perfiles.py` | Perfiles de costo de ff y τ; τ del modelo rígido elegido en entrenamiento | `perfiles.json` |
| 7 | `analisis_video.py` | Sincronización video ↔ corrida (correlación de rapidez) y respuesta libre en video | `video_llegadas.csv`, `video_corridas.csv` |
| 8 | `tray_video.py` | Error de posición en video de las trayectorias (modelo, inercia constante, rígido) | `tray_video.csv`, `tray_video_fallas.csv` |
| 9 | `modos.py` | Modos del modelo por postura, visibilidad en el giro y en el efector, carga de 100 g | `modos_modelo.json` |
| 10 | `estadistica.py` | ANOVA tipo III (log f_n y ζ; postura × nivel + sesión como bloque), IC95 % por celda, sensibilidad | `doe_anova.csv`, `doe_celdas.csv`, `doe_sensibilidad.csv` |
| 11 | `validacion.py` | Comparaciones pareadas en prueba: IC95 % de la diferencia y Wilcoxon | `validacion_pareada.csv` |
| 12 | `tabla_corridas.py` | Tabla de corridas y log de exclusiones | `data/runs.csv`, `data/exclusions.csv` |
| 13–18 | `fig1.py` … `fig6.py` | Figuras del manuscrito | `figures/` |

Módulos de apoyo: `indice.py` (índice de corridas, posturas, niveles, réplicas y reglas de exclusión), `imu_libre.py`
(detección de llegadas y ajuste), `video_lib.py` (carga y sincronización del video), `doe.py` (celdas y ANOVA),
`lib/modelo5b.py` (cinemática, M(q), simulación del servo), `lib/video_real.py` (rectificación y alineación del video),
`lib/identificar.py` (Levenberg–Marquardt), `lib/r5b/` (cinemática de la app, para el espacio de trabajo de la Fig. 1).

## Diccionario de `data/runs.csv`

| Columna | Significado |
|---|---|
| `run_id` | Fecha y hora de inicio de la corrida (reloj del PC); identifica la carpeta en `data/raw` |
| `session_folder` | S1 piloto y calibración (22 nov) · S2 bloque 1 del DOE (22 nov + repeticiones del 23 nov 15:38–15:42) · S3 bloque 2 (23 nov) · S4 diagnóstico (23 nov) |
| `doe_session` | Bloque del DOE (1 = S2, 2 = S3) |
| `pass`, `replicate` | Pase dentro de la sesión (1–2) y réplica del DOE (1–4) |
| `order_in_session` | Orden **ejecutado** dentro de la sesión (cronológico); el orden sorteado (semilla 20261122) está en `bitacora.xlsx` |
| `role` | `DOE`, `tap_test`, `validation_posture`, `trajectory_train`, `trajectory_test`, `payload_100g`, `payload_control_0g`, `locked_arms`, `pilot`, `pilot_tap`, `rest_noise`, `kinematic_calibration` |
| `posture`, `x_mm`, `y_mm` | Postura (A–D, V1, V2) y coordenadas comandadas |
| `level`, `delta_mm` | Nivel de excitación L1–L3 y Δ = 4 / 12 / 36 mm |
| `trajectory`, `vmax_mm_s`, `amax_mm_s2` | Trayectoria (C = cuadrado, O = círculo, velocidad/aceleración) |
| `payload_g` | Carga en el efector (100 g fijada con la garra y cinta) |
| `video_tracking_file`, `video_valid`, `video_exclusion_reason` | CSV de seguimiento asociado y si sus métricas de video se usan |
| `excluded`, `exclusion_reason` | Exclusión de la corrida completa (todos los sensores) |
| `fn_gyro_hz`, `zeta_gyro` | f_n y ζ de la corrida (mediana de llegadas), cuando aplica |

## Respuestas a las preguntas del docente

### 1. ¿Las 20 trayectorias de prueba o los 13 videos se usaron para algo durante el ajuste?

No para ajustar parámetros ni para elegir la estructura. El detalle:

- **División entrenamiento/prueba.** Quedó fijada en el plan del 22 nov, antes de tomar los datos (`docs/notas/plan_de_pruebas.md`):
  se ajusta con los cuadrados 100/1000 y 200/2000 de S2 y todo lo demás es prueba.
- **Estructura del modelo.** M(q) + lazo del servo (K, B, ff) + retardo viene de la etapa 2 del proyecto (13–15 nov), con datos
  anteriores al mantenimiento. No se cambió después de ver las corridas de prueba.
- **K, B y τ.** Se ajustan solo con las 4 corridas de entrenamiento. Lo mismo para el modelo de inercia constante.
- **ff = 0.5.** Se fijó porque el perfil de costo en entrenamiento es plano (+10 % entre 0 y 1) y 0.5 es el valor del driver (P02-03).
  Hay que declarar que se calcularon las métricas de prueba con ff libre y con ff fijo: dan 76.1 % y 76.0 %, así que la
  elección no favorece al modelo reportado.
- **Retardo del modelo rígido (85 ms).** Se buscó en una malla solo con las corridas de entrenamiento (`perfiles.py`).
- **Umbral de sincronización 0.75.** Se aplica solo a la respuesta libre en video (corridas del DOE y posturas de validación;
  Fig. 4c y f_n por video). No toca la validación de trayectorias, que alinea cada video por su cuenta. Se fijó después de
  ver la distribución de r (73 corridas): una sola está en 0.40 y la siguiente en 0.77. Es un corte por calidad de la
  sincronización, no por el resultado del modelo.
- **Círculo 16:15:55.** Se excluyó de las métricas de video después de calcularlas. Los tres modelos dan el mismo error
  (≈ 58 mm), así que es una falla de la alineación del video, no del modelo. Con esa corrida incluida (n = 14) la
  conclusión no cambia: la diferencia M(q) − inercia constante es 0.59 mm, M(q) es mejor en 11 de 14 y Wilcoxon da p = 0.013.
- **Seis trayectorias de S3 (pase 1).** No tienen métricas de video porque el marcador se detectó en el 46 % de los cuadros y
  la reconstrucción falla. Se registró antes de compararlas con el modelo.
- **Fig. 3.** El modelo se alinea con el video por un desfase por corrida estimado con el propio modelo, pero eso es solo para
  dibujar; no entra en ninguna métrica.

### 2. ¿Se corrigió la distorsión de la lente del celular?

No. Solo se rectificó el plano con el marcador (`lib/video_real.py`, `rectificar` y `a_plano`):

- Es una transformación proyectiva de 5 parámetros (escala, cizalla, relación de aspecto y dos términos de perspectiva).
  Se ajusta para que el cuadro de 20 mm tenga sus 4 lados y 2 diagonales correctos en todos los cuadros (error 0.12 mm RMS).
- Después se alinea al marco del robot con rotación, traslación y espejo.
- La distorsión radial queda dentro del error de la calibración con 18 puntos de reposo repartidos por el área de trabajo:
  1.15–1.17 mm RMS frente a la cinemática nominal.

Para f_n y ζ no importa (son tiempos). Para las amplitudes en mm y el RMSE de posición, ese 1.15 mm es la cota.
En el manuscrito conviene decir: "lens distortion was not modelled; its effect is bounded by the 1.15 mm calibration residual".

### 3. ¿Cuántas corridas se excluyeron en total?

Siete registros quedan fuera de todos los análisis, más ocho que solo pierden las métricas de video (`data/exclusions.csv`).

| Registro | Tipo | Motivo |
|---|---|---|
| 22 nov 12:50:34 | DOE D-L1 | interrumpida a los 5.5 s |
| 22 nov 12:50:56, 12:51:27 | DOE D-L2, D-L3 | postura corrida 4 mm a (0, 146) por la interrupción |
| 22 nov 12:51:49 | tap test D | misma postura corrida |
| 22 nov 13:06:13, 13:06:56 | V2, V1 | corridas a 100/1000 en vez de 300/2000 |
| 23 nov 15:38:47 | DOE D-L1 | repetición extra de una réplica válida; solo para sensibilidad |

El "six" del borrador contaba las corridas de excitación comandada: 3 del DOE + 2 de validación + la repetición extra.
La tabla del borrador agrupa filas (las dos de (0, 146) en una, el tap en esa misma), por eso se ven cinco.
Texto sugerido para el manuscrito:

> *Seven records were set aside before analysis: three DOE runs (one interrupted, two at a posture shifted by 4 mm),
> one tap record at the shifted posture, two validation runs executed at the wrong speed, and one extra repeat of a valid
> replicate, which was used only for the sensitivity analysis. Every affected cell was re-run. Eight further runs kept their
> gyroscope data but lost their video metrics (one synchronization failure, six runs with insufficient marker tracking,
> and one alignment failure).*

### 4. ¿Cómo se calculó la predicción de −8.1 % con 100 g en D?

Se registró el 23 nov a las 17:40, antes de la sesión 4 (`docs/notas/revision_S1_S2.md`), con el modelo **anterior**
K = diag(1.335, 0.865) N·m/rad (no con el identificado en este paper):

- Se calcularon los modos con y sin 0.100 kg puntuales en el eje del efector.
- La razón de frecuencias del modo que ve el giro fue 0.952 en A (−4.8 %) y 0.919 en D (−8.1 %).
- Esa razón se aplicó a la única réplica válida que había en ese momento a Δ2, la del pase 1 de S2: A 3.36 → 3.20 Hz y
  D 1.96 → 1.80 Hz. Las líneas de la Fig. 6b del borrador eran esos valores absolutos.

El docente tiene razón: con la línea base final (media DOE L2, n = 4) la misma razón da 3.22 Hz en A y 1.87 Hz en D.
La Fig. 6b ya se rehízo así (`figures/fig6_diagnostico.*`). Texto sugerido para el pie:

> *Dashed red: frequency ratio registered before the test (0.952 at A, 0.919 at D, computed with the earlier model)
> applied to the final DOE L2 mean.*

Con el modelo identificado en este paper la razón es casi igual: −4.6 % en A y −8.2 % en D (`modos_modelo.json`).

### 5. ¿Cómo se calculó la "visibilidad" de cada modo en el giróscopo?

En `scripts/modos.py`:

1. En cada postura se linealiza el modelo y se resuelve el problema de valores propios de
   A = [[0, I], [−M⁻¹K, −M⁻¹B]] (M = M(q), K y B identificados).
2. Para cada modo se toma la parte de articulaciones v ∈ ℂ² del vector propio, es decir las amplitudes complejas de q₁ y q₂.
3. La visibilidad en el giro es |J₃·v| / ‖v‖, donde J₃ = ∂φ₃/∂q es la fila del jacobiano del ángulo absoluto del distal
   derecho, que es lo que mide el giróscopo. Es adimensional (rad de distal por rad de articulación): cuánto del movimiento
   de ese modo ve la IMU.
4. Análogo para el efector: ‖J_ef·v‖ / ‖v‖ (m/rad), con J_ef el jacobiano de la posición del efector.

El "modo dominante en el giro" de la Fig. 5d es el de mayor visibilidad en el giro.

### 6. ¿Qué herramienta de IA se usó, con modelo y versión?

**Claude, de Anthropic**, en la aplicación Claude con ejecución de código y acceso a los archivos del equipo
(noviembre de 2026).

- La sesión que hizo el análisis final, las figuras y este repositorio corre con **Claude Opus 5.5**
  (identificador `claude-opus-5-5`).
- Si partes anteriores del proyecto se hicieron con otro modelo, el selector de modelo de cada conversación lo muestra;
  revísenlo antes de cerrar la declaración.

Se usó para escribir los scripts de procesamiento, identificación y estadística, hacer las figuras y redactar y editar el
texto. Los experimentos los diseñaron y ejecutaron los autores, que revisaron los resultados.

## Ensayos opcionales

Ver `docs/ensayos_opcionales.md`:

- golpe con los servos apagados y los brazos libres;
- golpe con los brazos sujetos y los servos activos;
- latencia del comando al movimiento.

Si se hacen, van en `data/raw/S5/Lecturas/` y quedan en `runs.csv` al correr `run_all.py`. Antes hay que agregar sus
roles en `indice.py`.

## Datos y licencia

Los datos personales de los autores (documento, firma) **no** van en este repositorio.
Licencia: [elegir; p. ej. CC BY 4.0 para datos y MIT para código].

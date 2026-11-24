# Etapa 2: modelo, identificación y EKF offline

`LEEME.md` describe la primera parte (modelo e identificación del 9 nov). El resto de los scripts de esta carpeta:

| Archivo | Qué hace |
|---|---|
| `modelo5b.py` | Cinemática, M(q) con las masas medidas, servo como lazo de posición con retardo, simulación |
| `identificar.py` | Ajuste de K, B, ff y τ con las corridas de la app (Levenberg–Marquardt) |
| `ekf5b.py` | EKF offline en Python; es la referencia con la que se validó `ekf_5b.h` del firmware 1.3 |
| `lqr5b.py`, `gen_lqr_tabla.py` | LQR con retardo y la tabla de ganancias por postura (`lqr_tabla.h`) |
| `lazo_sim.py` | Simulación del lazo cerrado con la ley de corrección |
| `video_seguir.py` | Seguimiento del marcador de 20 mm en los videos del celular |
| `video_real.py`, `video_comparar.py` | Rectificación del plano con el marcador, alineación al robot y comparación con la cinemática / el EKF |
| `calib_video.py` | Calibración cinemática con los reposos de la secuencia de 18 puntos |
| `figuras_video.py`, `pickplace_analizar.py` | Métricas con video: contorno, asentamiento, repetibilidad (ISO 9283) |

Los parámetros identificados aquí (9–13 nov) son anteriores al mantenimiento del 22 nov. Los vigentes son los del Paper 1 (`paper1-tecnologicas/results/identificacion_ff0.5.json`): K₁ = 1,29, K₂ = 1,06 N·m/rad, B₁ = 0,032, B₂ = 0,055 N·m·s/rad, τ = 25,7 ms con ff = 0,5.

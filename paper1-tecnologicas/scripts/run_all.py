"""Corre todo el análisis del Paper 1, de los datos crudos a las figuras y tablas.
    python run_all.py            (todo, ~15–30 min según el equipo)
    python run_all.py --desde 7  (retoma desde el paso 7)"""
import subprocess, sys, os, time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
L1 = "../data/raw/S1/Lecturas/"
CAL = [  # calibración cinemática con video (S1): cada secuencia de 18 puntos con su video
    ("calibracion_video.py", ["../data/raw/video_tracking/S1_20261122_103436_marcador.csv", L1 + "2026-11-22_10-35-48_Secuencia_de_18_puntos_con_pausa", "--salida", "../results/calib/secuencia1"]),
    ("calibracion_video.py", ["../data/raw/video_tracking/S1_20261122_103603_marcador.csv", L1 + "2026-11-22_10-37-10_Secuencia_de_18_puntos_con_pausa", "--salida", "../results/calib/secuencia2"]),
]
PASOS = [
    ("analisis_imu.py", {}),                    # 1. respuesta libre en el giro (f_n, ζ por llegada y por corrida)
    ("identificacion.py", {}),                  # 2. identificación con ff libre (para el perfil y la identificabilidad)
    ("identificacion.py", {"FF_FIJO": "0.5"}),  # 3. identificación reportada (ff fijo = P02-03 del driver)
    ("perfiles.py", {}),                        # 4. perfiles de costo de ff y τ; τ del modelo rígido (en entrenamiento)
    ("analisis_video.py", {}),                  # 5. sincronización y respuesta libre en video
    ("tray_video.py", {}),                      # 6. error de posición en video de las trayectorias
    ("modos.py", {}),                           # 7. modos del modelo, visibilidad, carga de 100 g
    ("estadistica.py", {}),                     # 8. ANOVA, celdas, sensibilidad
    ("validacion.py", {}),                      # 9. comparaciones pareadas (Wilcoxon)
    ("tabla_corridas.py", {}),                  # 10. data/runs.csv y data/exclusions.csv
    ("fig1.py", {}), ("fig2.py", {}), ("fig3.py", {}), ("fig4.py", {}), ("fig5.py", {}), ("fig6.py", {}),
]
desde = int(sys.argv[sys.argv.index("--desde") + 1]) if "--desde" in sys.argv else 1
TODO = [(s, a, {}) for s, a in CAL] + [(s, [], e) for s, e in PASOS]   # pasos 1–2: calibración; 3–18: resto
for i, (s, args, env) in enumerate(TODO, 1):
    if i < desde: continue
    if "--salida" in args: os.makedirs(args[args.index("--salida") + 1], exist_ok=True)
    t = time.time(); print("== %2d %s %s" % (i, s, env or ""), flush=True)
    subprocess.run([sys.executable, s] + args, check=True, env={**os.environ, **env})
    print("   %.0f s" % (time.time() - t), flush=True)

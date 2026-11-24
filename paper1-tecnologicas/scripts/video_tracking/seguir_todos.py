"""
Sigue el marcador del efector en TODOS los videos del Paper 1.

Busca los .mp4 dentro de las carpetas S*/Videos (o videos) de la carpeta de resultados y guarda, para cada
uno, un CSV pequeño en <carpeta>/marcador/. Si se interrumpe, al volver a correrlo sigue con los que faltan.

Uso:  python seguir_todos.py "C:\\Universidad\\Servo\\Resultados Paper 1"  [--procesos 3]
"""
import argparse
import glob
import multiprocessing as mp
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import video_seguir  # noqa: E402


def trabajo(args):
    video, out = args
    nombre = os.path.basename(video)
    t0 = time.time()

    def progreso(i, ti):
        print("   %s: %5.0f s de video procesados (%.0f cuadros/s)" % (nombre, ti, (i + 1) / max(time.time() - t0, 1e-3)),
              flush=True)

    try:
        tmp = out + ".parcial"
        res = video_seguir.seguir(video, out=tmp, progreso=progreso)
        os.replace(tmp, out)
        return "OK    %-28s %6d cuadros, marcador en %5.1f %%, %4.0f min" % (nombre, res[1], 100 * res[2],
                                                                            (time.time() - t0) / 60)
    except Exception as e:  # sigue con los demás
        return "ERROR %-28s %s" % (nombre, e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("carpeta", help="carpeta 'Resultados Paper 1'")
    ap.add_argument("--procesos", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args()
    raiz = os.path.abspath(a.carpeta)
    salida = os.path.join(raiz, "marcador")
    os.makedirs(salida, exist_ok=True)
    videos = sorted(set(glob.glob(os.path.join(raiz, "S*", "[Vv]ideos", "*.mp4")) +
                        glob.glob(os.path.join(raiz, "S*", "[Vv]ideos", "*.MP4"))))
    tareas = []
    for v in videos:
        sesion = os.path.basename(os.path.dirname(os.path.dirname(v)))
        out = os.path.join(salida, "%s_%s_marcador.csv" % (sesion, os.path.splitext(os.path.basename(v))[0]))
        if not os.path.exists(out):
            tareas.append((v, out))
    print("Videos encontrados: %d. Ya procesados: %d. Por procesar: %d, con %d procesos a la vez."
          % (len(videos), len(videos) - len(tareas), len(tareas), a.procesos), flush=True)
    if not tareas:
        print("No hay nada pendiente. Los resultados están en:", salida)
        return
    log = open(os.path.join(salida, "registro.txt"), "a", encoding="utf-8")
    log.write("\n=== %s ===\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
    t0 = time.time()
    with mp.Pool(a.procesos) as pool:
        for k, r in enumerate(pool.imap_unordered(trabajo, tareas), 1):
            linea = "[%d/%d] %s" % (k, len(tareas), r)
            print(linea, flush=True)
            log.write(linea + "\n")
            log.flush()
    print("\nListo en %.0f min. Los resultados están en: %s" % ((time.time() - t0) / 60, salida))


if __name__ == "__main__":
    mp.freeze_support()
    main()

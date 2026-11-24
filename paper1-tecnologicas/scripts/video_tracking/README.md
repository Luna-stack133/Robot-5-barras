# Seguimiento del marcador en los videos (paso previo, se corre en el PC con los .mp4)

Convierte cada video del celular (60 fps, 1080 × 1920) en un CSV con la posición del marcador de 20 mm cuadro a cuadro
(centro y 4 esquinas en píxeles, con refinamiento subpíxel, más la estabilización del fondo). Esos CSV son los que están
en `data/raw/video_tracking/` y los que usa el resto del análisis. Los videos originales no están en el repositorio.

- `video_seguir.py`: seguimiento de un video. Lee con ffmpeg si está instalado y si no con OpenCV (dan las mismas posiciones).
- `seguir_todos.py`: recorre `S1…S4/Videos/*.mp4` de la carpeta de resultados, en paralelo y retomable; escribe `marcador/<sesión>_<video>_marcador.csv`.
- `ejecutar.bat`: crea un entorno, instala `numpy` y `opencv-python` y corre todo (Windows). Uso: doble clic, o arrastrar la carpeta de resultados encima.

```bash
pip install numpy opencv-python
python seguir_todos.py "C:\Universidad\Servo\Resultados Paper 1" --procesos 3
```

Con unos 40 videos tarda 1–3 h. `registro.txt` (en `data/raw/video_tracking/`) guarda el porcentaje de cuadros con marcador de cada video.

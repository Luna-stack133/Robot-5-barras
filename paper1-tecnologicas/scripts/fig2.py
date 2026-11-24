from rutas import *
from estilo import *
from PIL import Image
import numpy as np
from matplotlib.patches import FancyBboxPatch
im=np.asarray(Image.open(os.path.join(DATOS,"frames","cuadro_S2_C.jpg")))
fig=plt.figure(figsize=(7.16,3.6))
a=fig.add_axes([0.0,0.02,0.24,0.93]); a.imshow(im[:1150]); a.axis("off"); a.set_title("(a) Camera view, 60 fps",fontsize=7.5,loc="left")
# recuadro con zoom del marcador (centro aprox en x=520,y=540 de 1080x1920)
cx,cy=515,545
a.add_patch(plt.Rectangle((cx-90,cy-90),180,180,fill=False,ec="#e34948",lw=1))
z=fig.add_axes([0.14,0.255,0.1,0.17]); z.imshow(im[cy-90:cy+90,cx-90:cx+90]); z.set_xticks([]); z.set_yticks([])
for s in z.spines.values(): s.set_edgecolor("#e34948"); s.set_linewidth(1)
z.set_title("20 mm marker",fontsize=6,pad=1)
b=fig.add_axes([0.27,0.0,0.73,1.0]); b.set_xlim(0,100); b.set_ylim(0,50); b.axis("off")
b.text(0,48.5,"(b) Measurement, synchronization, identification and validation chain",fontsize=7.5,fontweight="bold",va="top")
def caja(x,y,w,h,t,sub="",c="#f1f0ec",ec=INK):
    b.add_patch(FancyBboxPatch((x,y),w,h,boxstyle="round,pad=0.4,rounding_size=1.2",fc=c,ec=ec,lw=0.6))
    b.text(x+w/2,y+h-1.6,t,ha="center",va="top",fontsize=6.6,fontweight="bold")
    if sub: b.text(x+w/2,y+h-5.0,sub,ha="center",va="top",fontsize=5.8,color="#3d3d3a",linespacing=1.25)
def fl(x1,y1,x2,y2): b.annotate("",xy=(x2,y2),xytext=(x1,y1),arrowprops=dict(arrowstyle="-|>",lw=0.6,color=INK,shrinkA=0,shrinkB=0,mutation_scale=7))
# fila superior: robot / registro
caja(1,33,22,11,"ESP32-S3 (fw 1.3)","STEP/DIR to both servos\nIMU loop 100 Hz (BNO055)")
caja(27,33,22,11,"R5B Lab app log","command q_c(t), telemetry,\ngyro/accel, run ID, metadata")
caja(53,33,22,11,"Phone video 60 fps","fixed tripod, zoom 0.9,\none video per posture visit")
caja(79,33,20,11,"Marker tracking","sub-pixel square corners,\nbackground stabilization")
fl(23,38.5,27,38.5); fl(75,38.5,79,38.5)
# fila media: procesamiento
caja(79,17,20,12,"Plane rectification","marker side error\n0.12 mm RMS; rest\nnoise 0.02–0.03 mm")
caja(53,17,22,12,"Camera / kinematic\ncalibration","18 rest points ×2:\n1.15 mm RMS (nominal\ngeometry, post-maint.)")
caja(27,17,22,12,"Run ↔ video sync","speed cross-correlation\n(median r = 0.84; 1 run\nwith r < 0.75 dropped)")
caja(1,17,22,12,"Free-response metrics","f_n, ζ per arrival (damped\nfit, IMU and video);\npeak, settling ±1/±0.5 mm")
fl(89,33,89,29); fl(79,23,75,23); fl(53,23,49,23); fl(38,33,38,29); fl(27,23,23,23)
# fila inferior: modelo
caja(1,1,30,12,"DOE statistics","4 postures × 3 levels × n = 4\nsession as block; IC95 %,\nη²p; exclusion log")
caja(35,1,30,12,"Identification (train)","M(q)q̈+Cq̇ = K[q_c(t−τ)−q]\n+B[ff q̇_c(t−τ)−q̇]; 4 squares\n(session 1), LM fit to gyro")
caja(69,1,30,12,"Validation (test)","20 unseen trajectories (gyro),\n13 with video; validation\npostures; 100 g payload",c="#e9f1fb",ec="#2a78d6")
fl(12,17,12,13); fl(38,17,45,13); fl(65,7,69,7)
guardar(fig,"fig2_cadena_medicion")

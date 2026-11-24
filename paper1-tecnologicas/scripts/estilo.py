from rutas import *
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({"font.family":"DejaVu Sans","font.size":8,"axes.titlesize":8.5,"axes.labelsize":8,
    "xtick.labelsize":7,"ytick.labelsize":7,"legend.fontsize":7,"axes.spines.top":False,"axes.spines.right":False,
    "axes.linewidth":0.6,"xtick.major.width":0.6,"ytick.major.width":0.6,"lines.linewidth":1.2,"axes.titleweight":"bold",
    "axes.titlelocation":"left","savefig.dpi":300})
COL={"A":"#2a78d6","B":"#eb6834","C":"#1baf7a","D":"#eda100","V1":"#6250d6","V2":"#e87ba4"}
MK={"A":"o","B":"s","C":"^","D":"D","V1":"v","V2":"P"}
LV={"L1":"#86b6ef","L2":"#2a78d6","L3":"#104281"}
INK="#1f1f1d"; GRAY="#8a8a85"; GRID="#e6e5e0"
OUT=FIGS
import os; os.makedirs(OUT,exist_ok=True)
def guardar(fig,nombre):
    fig.savefig(f"{OUT}/{nombre}.png",dpi=300,bbox_inches="tight"); fig.savefig(f"{OUT}/{nombre}.pdf",bbox_inches="tight")
def ejes(ax):
    ax.grid(color=GRID,lw=0.5); ax.set_axisbelow(True)

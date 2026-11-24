# Paper 1 — Revisión de las sesiones 1 y 2 (22 nov 2026; revisado el 23 nov)

## Estado
- **S1 completa:** 5 reposos, calibración ×2, piloto (A y D, Δ = 4 / 12 / 36 mm) y golpes de práctica.
- **S2 completa:** 2 pasadas, 24 excitaciones, 8 golpes, V1/V2 y 12 trayectorias. Orden sorteado cumplido.
- **Firmware: 1.3** con la corrección apagada.
- **Reloj del celular ~22 s atrasado** respecto al PC. La sincronización se hace por correlación.

## Corridas que hay que repetir o que quedan excluidas
- 12:50:34 (D, Δ1): interrumpida a los 5.5 s → **excluida**. La de 12:50:23 está completa y es válida.
- La interrupción dejó el robot en P + Δ1 = (0, 146), así que D Δ2, Δ3 y los golpes de la pasada 2 se hicieron 4 mm corridos → **repetir la visita a D**.
- V1 y V2 de la pasada 2 (13:06) se corrieron a 100/1000 en vez de 300/2000 → **repetir**.

## Resultados preliminares (IMU, llegadas a P; mediana por corrida, pasada 1 / pasada 2)
Frecuencia natural en Hz:

| Postura | Δ1 | Δ2 | Δ3 | Golpes |
|---|---|---|---|---|
| A | 3.75 / 3.77 | 3.31 / 3.41 | 3.21 / 3.33 | 3.31 / 3.51 |
| B | 3.88 / 3.71 | 2.82 / 2.76 | 2.67 / 2.55 | 2.45 / 2.46 |
| C | 3.59 / 3.46 | 2.32 / 2.33 | 1.86 / 1.86 | 1.92 / 1.98 |
| D | 2.89 / 2.95 | 1.96 / (1.87*) | 1.63 / (1.64*) | 1.72 / (1.74*) |

\* Medidas en (0, 146).

- Repetibilidad entre pasadas: diferencias de 0–4 % en f_n. Los ajustes dan R² de 0.90–0.99.
- f_n baja con la amplitud en todas las posturas (C: 3.5 → 2.3 → 1.9 Hz): rigidez no lineal (ablandamiento).
- ζ por postura: B 0.10–0.13; A y C 0.16–0.26; D 0.2–0.37.
- Ruido del giróscopo en reposo: 0.12–0.16 °/s de desviación estándar.
- Pre/post mantenimiento: los golpes del 7 nov se hicieron con otra sintonía (Kp 20), no comparables. Con la sintonía congelada (V1, 15 nov): 2.22–2.32 Hz antes y 2.33–2.34 Hz después.
- Asentamiento: con Δ3 en C y D, |gz| < 2 °/s a los 1.4–1.5 s, dentro de la pausa de 2 s.

## Predicción registrada ANTES de la sesión 4 (23 nov 2026, 17:40): carga de 100 g en el efector
- **Método:** modelo M(q) de `modelo5b.py` con el modelo previo K = diag(1.335, 0.865) N·m/rad, más 0.100 kg puntual en el eje del efector. Se calcula la razón de frecuencias de los modos (con / sin carga) y se aplica al modo que ve el giróscopo.
- **Razón de frecuencias:** A 0.952 (modo visible en gz: el de ~3.5 Hz); B 0.933; C 0.92–0.93; D 0.919 (modo de ~1.3–2 Hz).
- **Predicción con Δ2**, aplicada a la única réplica válida disponible ese momento (S2, pasada 1): A 3.36 → **3.20 Hz**; D 1.96 → **1.80 Hz**.

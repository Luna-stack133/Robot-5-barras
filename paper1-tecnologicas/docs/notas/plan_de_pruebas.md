# Paper 1 (identificación dinámica, TecnoLógicas) — plan de pruebas (22 nov 2026)

- Se empieza por el Paper 1, después del mantenimiento (prisioneros nuevos, cable reubicado). Todo lo anterior queda como dato *pre-mantenimiento*.
- El reductor 6:1 no se monta hasta cerrar las campañas del Paper 1 y el Paper 2 (transmisión directa).
- Firmware en modo Apagada (sin corrección) y prealimentación en 0; app R5B Lab 0.4.1, con el botón "Cargar excitación aquí". (Nota posterior: el firmware cargado fue el 1.3.)

## Diseño
- Posturas A (75, 280), B (170, 220), C (75, 200), D (0, 150); validación V1 (25, 240), V2 (150, 250).
- Excitación comandada: P → P + Δ (−y) → P, 5 ciclos, pausa de 2 s, 300 mm/s y 2000 mm/s². Δ propuestos 4 / 12 / 36 mm; se fijan en el piloto de S1.
- 4 posturas × 3 niveles × 4 réplicas = 48 corridas, más 16 de golpes a mano como contraste, 8 de validación y 24 de trayectorias.
- Trayectorias: cuadrados 100/200, 100/1000, 100/2000, 200/2000 y círculos 100/1000, 200/2000, ×4 réplicas. **El ajuste usa solo los cuadrados 100/1000 y 200/2000 de S2; todo lo demás es prueba** (división fijada antes de tomar los datos).
- Sesiones: S1 verificación (reposo, calibración con video ×2, piloto); S2 y S3 bloque principal en días distintos, con orden sorteado (semilla 20261122); S4 diagnóstico: carga en A y D, brazos bloqueados con los servos apagados y latencias con LED (opcional).
- Unidad experimental = una corrida; las 5 llegadas de cada corrida son observaciones anidadas. Bitácora con una fila por corrida y lista de exclusiones.
- El 145 Hz no se afirma.

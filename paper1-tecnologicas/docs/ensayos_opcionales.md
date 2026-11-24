# Ensayos opcionales sugeridos por el docente (solo si son seguros y rápidos)

Se registran con la app R5B Lab 0.4.1 igual que S4, en una carpeta nueva `data/raw/S5/Lecturas/`, con una fila en
`bitacora.xlsx` por corrida. Después se corre `python scripts/run_all.py` y quedan en `data/runs.csv`.

## E1. Golpe con los servos apagados y los brazos libres
- Postura (75, 284), servos deshabilitados (sin par), brazos **sin** utillaje. 4 golpes suaves en el efector, uno por grabación.
- Para qué: separa la inercia + fricción de la mecánica sola, sin el lazo del servo y sin el bloqueo.
- Cuidado: con los servos apagados el brazo cae o se desplaza; sujetar el efector con la mano antes y después del golpe.

## E2. Golpe con los brazos sujetos por el utillaje y los servos activos
- Misma postura, utillaje puesto, servos habilitados y quietos. 4 golpes.
- Para qué: completa el 2 × 2 (servo on/off × brazos libres/sujetos) y aísla la rigidez del utillaje y de los eslabones.
- Cuidado: si el driver da alarma por error de posición o se oye chillido, parar y no repetir.

## E3. Latencia del comando al movimiento
- Opción sencilla: grabar con el celular a 240 fps un LED que el firmware enciende en el primer pulso STEP, junto con el marcador.
  El retardo es el tiempo entre el LED y el primer cuadro en que el marcador se mueve más de 0.1 mm.
- Opción con lo que hay: 10 escalones pequeños (Δ = 4 mm) desde reposo; comparar la marca de tiempo del comando en el log
  con el inicio del movimiento en el giróscopo (umbral 3 × ruido, 0.5 °/s). Da el retardo comando → giro sin separar la latencia
  de la IMU.

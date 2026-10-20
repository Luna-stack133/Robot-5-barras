# Etapa 0 — Poner a funcionar la MPU6050 y caracterizarla

Robot paralelo de 5 barras · Técnicas de Control

El objetivo de esta etapa es confirmar que la IMU funciona bien, que sus ejes y signos están claros y medir su ruido y su sesgo reales. Esos números son los que después alimentan el EKF (matriz R y valores iniciales del sesgo).

## Qué hay en esta carpeta

| Archivo | Para qué sirve |
|---|---|
| `imu_etapa0/imu_etapa0.ino` | Programa para la ESP32-S3. Lee la IMU a 200 Hz y envía los datos por serial |
| `etapa0_imu.py` | Script de Python para registrar los datos en un CSV y analizarlos |

El programa de la ESP32 es independiente del código del robot. Primero se verifica la IMU sola y más adelante se integra.

## 1. Conexión

| GY-521 | ESP32-S3 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 8 |
| SCL | GPIO 9 |
| INT | GPIO 10 |
| AD0 | GND o sin conectar |
| XDA, XCL | sin conectar |

Si los GPIO 8, 9 o 10 ya los usan para PUL/DIR, cámbienlos al inicio del `.ino` (constantes `PIN_SDA`, `PIN_SCL`, `PIN_INT`). Eviten los pines 0, 3, 45 y 46 (arranque), 19 y 20 (USB), 43 y 44 (serial) y 35 a 37 si su placa es N8R8 o N16R8.

Si no conectan INT el programa igual funciona: lo detecta y lee por sondeo. Pero las marcas de tiempo quedan con hasta 0.5 ms de error, así que es mejor conectarlo.

Montaje en el robot: la IMU va atornillada y plana sobre el eslabón distal, cerca del efector, con el eje X de la placa a lo largo del eslabón. Cable flexible y corto, con SDA y SCL trenzados cada uno con un GND, con holgura en las articulaciones y lejos de los cables de potencia de los servos.

## 2. Cargar el programa

1. En el Arduino IDE abran `imu_etapa0/imu_etapa0.ino`.
2. Placa: **ESP32S3 Dev Module**.
3. Si conectan la placa por el puerto marcado **USB** (el nativo), activen *Tools → USB CDC On Boot → Enabled*. Si la conectan por el puerto marcado **COM** o **UART**, déjenlo deshabilitado.
4. Carguen el programa y abran el monitor serial a **921600 baudios**.

El programa no necesita librerías externas: configura la IMU directamente por registros. Se verificó que compila sin advertencias con el core de ESP32 2.0.17 y con el 3.3.12.

## 3. Prueba 1 — Arranque

Con la IMU quieta, presionen el botón RESET (o EN) de la placa. En el monitor deben ver algo así:

```
# Escaneando bus I2C...
#   Dispositivo en 0x68  <- probablemente la IMU
# WHO_AM_I = 0x68 -> MPU6050 (correcto)
# INT conectado en GPIO 10 (40 pulsos en 200 ms).
# Autoprueba: mantengan la IMU quieta durante 1 segundo...
#   Acelerometro medio [g]:   x=+0.012  y=-0.020  z=+1.003
#   Magnitud |a| = 1.003 g  (bien, debe ser cercana a 1)
#   Giroscopio medio [deg/s]: x=+0.85  y=-0.47  z=-1.15  (sesgo en reposo)
```

| Si ven esto | Significa |
|---|---|
| "Ninguno" en el escaneo | Problema de cableado: revisen VCC, GND, y que SDA y SCL no estén invertidos |
| WHO_AM_I distinto de 0x68 | Es un clon o un chip parecido. El programa continúa; anoten el valor |
| "No se detectaron pulsos en GPIO 10" | INT no está conectado o está en otro pin |
| \|a\| lejos de 1 g | Revisen que la IMU esté quieta; si persiste, anótenlo |
| Sesgo del giroscopio de 0.5 a 3 °/s | Normal en la MPU6050 (la hoja de datos admite hasta ±20 °/s). El EKF lo va a estimar |

## 4. Prueba 2 — Ejes y signos

Usen el Serial Plotter del Arduino IDE (*Tools → Serial Plotter*, 921600 baudios). Escriban `a` y Enter para ver el acelerómetro, o `g` para el giroscopio. Con `s` se detiene.

**IMU suelta, en la mano:**

| Movimiento | Lo que deben ver |
|---|---|
| Plana sobre la mesa, chip hacia arriba | az ≈ +1 g; ax y ay ≈ 0 |
| De canto, con la flecha X de la placa apuntando hacia arriba | ax ≈ +1 g |
| De canto, con la flecha Y apuntando hacia arriba | ay ≈ +1 g |
| Plana, girándola sobre la mesa en sentido antihorario (vista desde arriba) | gz positivo mientras gira |
| Girándola alrededor del eje X, y luego del Y | gx y luego gy responden |

**IMU montada en el robot:** con los drivers deshabilitados, muevan el robot despacio con la mano (o con su código, a baja velocidad).

- Cuando el eslabón distal gira en sentido antihorario visto desde arriba, gz debe ser positivo.
- **Anoten cómo se relaciona ese signo con la convención de ángulos de su cinemática.** Si en su modelo el ángulo del eslabón distal crece en sentido antihorario, los signos coinciden; si no, hay que invertir el signo de gz. Esto es indispensable para la etapa 1.
- Mientras el robot se mueve en el plano, gx y gy deberían quedarse cerca de cero y az cerca de 1 g. Si gx o gy se mueven bastante, el eslabón está cabeceando fuera del plano.

## 5. Prueba 3 — Registros en reposo

Esta es la medición principal de la etapa. Cierren el monitor y el plotter del Arduino IDE antes de usar Python (si no, el puerto queda ocupado).

Instalen las dependencias una sola vez:

```
pip install pyserial numpy matplotlib
```

Busquen el puerto de la ESP32:

```
python etapa0_imu.py puertos
```

Hagan **dos registros**, con la IMU montada en el robot:

**Registro A — servos apagados.** Dejen la ESP32 y la IMU encendidas 5 minutos antes, para que se estabilice la temperatura. Luego:

```
python etapa0_imu.py registrar --puerto COM5 --minutos 10 --salida reposo_servos_off.csv
```

**Registro B — servos energizados pero quietos**, sosteniendo su posición:

```
python etapa0_imu.py registrar --puerto COM5 --minutos 10 --salida reposo_servos_on.csv
```

Durante cada registro nadie toca la mesa ni el robot. Si pueden, hagan el registro A de 20 minutos: la desviación de Allan sale más confiable.

Analicen cada archivo:

```
python etapa0_imu.py analizar reposo_servos_off.csv
python etapa0_imu.py analizar reposo_servos_on.csv
```

Cada análisis imprime un reporte y guarda:

- `<nombre>_calibracion.json`: sesgos, varianzas R para el EKF, densidad de ruido, parámetros de Allan y deriva térmica.
- `<nombre>_series.png`, `_psd.png`, `_allan.png`, `_temperatura.png`: gráficas.

## 6. Cómo leer los resultados

| Resultado | Qué esperar | Si sale mal |
|---|---|---|
| Muestras perdidas y huecos | 0 | Revisen el cable USB y que ningún otro programa use el puerto |
| Magnitud \|a\| | Entre 0.92 y 1.08 g | Un valor constante fuera de rango es un error de escala; se corrige calibrando |
| Inclinación (roll, pitch) | Menos de 3° | La IMU no quedó plana: vuelvan a montarla o lo compensamos |
| Densidad de ruido del acelerómetro | Cerca de 400 µg/√Hz | Si es más de 3 veces mayor: vibración o ruido eléctrico |
| Densidad de ruido del giroscopio | Cerca de 5 mdps/√Hz | Igual que arriba |
| Sesgo del giroscopio | Hasta unos pocos °/s | Normal en la MPU6050. Lo importante es que sea estable |
| Deriva térmica | Unas centésimas de °/s por °C | Muestra por qué hay que calentar la IMU antes de cada prueba |

**La comparación entre A y B es la parte más informativa.** Si el ruido de B es claramente mayor que el de A, los servos energizados están generando vibración o interferencia eléctrica. Eso hay que saberlo antes de diseñar el EKF, porque ese ruido extra es justo lo que el filtro tendrá que manejar.

## 7. Lo que queda al terminar la etapa 0

- [ ] La IMU arranca, responde en 0x68 y la autoprueba da \|a\| ≈ 1 g
- [ ] Ejes y signos verificados, y anotada la relación entre gz y su convención de ángulos
- [ ] Registro A (servos apagados) y registro B (servos energizados), cada uno con su JSON y sus gráficas
- [ ] Cero muestras perdidas en ambos registros
- [ ] Anotados el valor de WHO_AM_I y los pines usados

Con los dos JSON y las gráficas pasamos a la etapa 1: registrar la IMU mientras el robot ejecuta trayectorias y compararla con lo que predice la cinemática.

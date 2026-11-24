# Etapa 0 — Conocer, calibrar y caracterizar la BNO055

Robot paralelo de 5 barras · Técnicas de Control

En esta etapa se comprueba que la BNO055 funciona, se aprende qué entrega cada modo, se calibra, se guarda la calibración y se miden su ruido y su sesgo reales. Esos números alimentan después el EKF.

## Qué hay en esta carpeta

| Archivo | Para qué sirve |
|---|---|
| `bno055_etapa0/bno055_etapa0.ino` | Programa para la ESP32-S3. Habla con la BNO055 por registros, sin librerías externas |
| `bno055_etapa0.py` | Script de Python para registrar los datos en un CSV y analizarlos |

El programa es independiente del firmware del robot: primero se conoce el sensor solo y más adelante se integra.

## 0. Qué es la BNO055 y qué la diferencia de la MPU6050

La BNO055 tiene tres sensores (acelerómetro, giroscopio y magnetómetro) y un procesador propio que los fusiona. Además de los datos de cada sensor, entrega la orientación ya calculada: cuaternión, ángulos de Euler, gravedad y aceleración lineal sin gravedad. Todo sale a 100 Hz.

Tiene varios modos de trabajo. En esta etapa se usan tres:

| Modo | Usa | Entrega | Para qué nos sirve |
|---|---|---|---|
| **IMU** (el recomendado) | acelerómetro + giroscopio | todo, con rumbo **relativo** (arranca en 0) | modo de trabajo en el robot: el magnetómetro no sirve cerca de los servos |
| **NDOF** | los tres sensores | todo, con rumbo **absoluto** (norte magnético) | solo para comprobar que el magnetómetro se perturba cerca de los motores |
| **AMG** | los tres, sin fusión | solo datos crudos | comparar con lo que hace la fusión |

Lo que necesita el EKF del proyecto son sobre todo la velocidad angular `gz` del giroscopio y la aceleración en el plano (`ax`, `ay`). El rumbo de la fusión sirve como referencia para comparar.

Hay dos detalles de resolución que conviene saber antes de mirar los datos:
- **El giroscopio sale en pasos de 1/16 °/s (0.0625 °/s).** Las gráficas se ven "escalonadas"; es normal.
- **El acelerómetro sale en pasos de 0.01 m/s².**

## 1. Conexión

| BNO055 | ESP32-S3 |
|---|---|
| VIN (o 3V3) | 3V3 |
| GND | GND |
| SDA | GPIO 8 |
| SCL | GPIO 9 |
| ADR | GND o sin conectar (dirección 0x28) |
| PS0, PS1 | GND o sin conectar (modo I²C) |
| INT, RST | sin conectar |

- Si los GPIO 8 y 9 ya están ocupados, cámbienlos al inicio del `.ino` (`PIN_SDA`, `PIN_SCL`). No usen los pines 4 a 7 (pulsos de los servos), 0, 3, 45 y 46 (arranque), 19 y 20 (USB) ni 43 y 44 (serial).
- Cables cortos, con SDA y SCL trenzados cada uno con un GND.

## 2. Cargar el programa

1. En el Arduino IDE abran `bno055_etapa0/bno055_etapa0.ino`.
2. Placa: **ESP32S3 Dev Module**.
3. Si conectan la placa por el puerto marcado **USB** (el nativo), activen *Tools → USB CDC On Boot → Enabled*. Por el puerto **COM** o **UART**, déjenlo deshabilitado.
4. Carguen el programa y abran el monitor serie a **921600 baudios**, con fin de línea "Nueva línea".

## 3. Prueba 1 — Arranque

Con la IMU quieta, presionen RESET. Deben ver algo así:

```
# Escaneando bus I2C...
#   Dispositivo en 0x28  <- probablemente la BNO055
# CHIP_ID=0xA0 (correcto) | ACC=0xFB MAG=0x32 GIR=0x0F | firmware 0x0311 | bootloader 0x15
#   Autoprueba = 0x0F -> acel OK, mag OK, gir OK, MCU OK
#   Reloj: cristal externo de 32.768 kHz
# Calibracion guardada: no hay. Calibre (comando c) y guardela (comando s).
# Modo: IMU (acelerometro + giroscopio, sin magnetometro)
#   Estado del sistema: 5 (fusion en marcha) | error: 0 (sin error)
#   Temperatura: 27 C
```

| Si ven esto | Significa / qué hacer |
|---|---|
| No aparece nada en 0x28 ni 0x29 | Cableado: VIN, GND, SDA/SCL invertidos, o PS0/PS1 no están en GND |
| Autoprueba distinta de 0x0F | Un sensor interno falló. Anoten cuál y repitan con el comando `t` |
| Estado distinto de 5, o error distinto de 0 | Pongan `USAR_CRISTAL_EXTERNO` en 0 (su módulo puede no tener cristal) y vuelvan a cargar |
| Errores de lectura intermitentes | Bajen `I2C_HZ` a 100000 y acorten los cables |

## 4. Prueba 2 — Ver los datos y entender los ejes

Usen el Serial Plotter (*Tools → Serial Plotter*, 921600 baudios). Escriban una letra y Enter: `a` acelerómetro, `g` giroscopio, `e` ángulos de Euler, `l` aceleración lineal. Con `q` se detiene.

**IMU suelta, en la mano:**

| Situación | Lo que deben ver |
|---|---|
| Plana sobre la mesa, chip hacia arriba | az ≈ +9.8 m/s²; ax y ay ≈ 0 |
| De canto, con la flecha X de la placa hacia arriba | ax ≈ +9.8 m/s² |
| De canto, con la flecha Y hacia arriba | ay ≈ +9.8 m/s² |
| Quieta | gx, gy, gz cerca de 0 (saltan de a 0.0625 °/s) |
| Plana, girándola sobre la mesa en sentido antihorario, vista desde arriba | **gz positivo** mientras gira |
| La misma rotación, viendo el rumbo (`e`) | el rumbo **cambia**. Anoten si sube o baja: en la BNO055 el rumbo suele crecer en sentido horario, al revés que `gz` |
| Quieta, aceleración lineal (`l`) | lx, ly, lz ≈ 0. Al deslizarla sobre la mesa aparece la aceleración del movimiento |

**Anoten la relación de signos entre `gz`, el rumbo y la convención de ángulos de `kinematics_5bar.h`.** Si los ángulos del robot crecen en sentido antihorario y `gz` también, coinciden. Si no, habrá que invertir un signo. Esto es indispensable para la etapa 1.

## 5. Prueba 3 — Calibración

La BNO055 se calibra sola mientras funciona, pero cada vez que se apaga la olvida. Por eso el procedimiento es: calibrar una vez, guardar los valores en la ESP32 (comando `s`) y que el programa los cargue en cada arranque.

Escriban `c` para ver el estado cada medio segundo:

```
CAL sistema=3  giroscopio=3  acelerometro=3  magnetometro=0 | IMU | estado 5 error 0 | T=27 C
```

Cada sensor va de 0 (sin calibrar) a 3 (calibrado). Calibren con la IMU **suelta, antes de montarla**: así es fácil ponerla en todas las posiciones.

1. **Giroscopio:** déjenla quieta sobre la mesa 3 a 5 segundos. Debería llegar a 3 enseguida.
2. **Acelerómetro**, el más lento: pónganla en **6 posiciones distintas**, quieta unos 3 segundos en cada una y moviéndola despacio entre ellas: chip arriba, chip abajo, y sobre cada uno de sus cuatro cantos. Apóyenla contra algo para que quede firme. Si no llega a 3, agreguen posiciones intermedias inclinadas unos 45°.
3. **Magnetómetro** (solo en modo NDOF, comando `n`): muévanla en el aire dibujando un "8" en varias orientaciones, lejos de los motores y de objetos de hierro.
4. Cuando giroscopio y acelerómetro marquen 3, escriban **`s`** para guardar. Con `o` pueden ver los offsets guardados. Anótenlos en la bitácora.
5. Presionen RESET: el programa debe decir "Calibracion guardada: cargada en el sensor". Es normal que el estado marque 0 unos segundos, hasta que el sensor vuelva a verificar.

Los offsets son del sensor, no del montaje, así que siguen valiendo después de atornillarlo al robot. Vuelvan a calibrar si cambian de módulo o si los valores de reposo se ven raros.

## 6. Prueba 4 — ¿Sirve el magnetómetro en el robot?

Esta prueba justifica, con datos, por qué el robot trabaja en modo IMU.

1. Con la IMU montada en el robot, pongan el modo NDOF (`n`) y miren el rumbo (`e`).
2. Con los servos apagados, anoten el rumbo. Energicen los servos y muevan el robot con JOG, despacio.
3. Repitan en modo IMU (`i`).

Lo esperado: en NDOF el rumbo salta o se desvía al acercarse a los motores o al energizarlos, por los imanes del motor, las corrientes y las piezas de acero. En IMU el rumbo solo sigue el giro real. Si en NDOF el rumbo se mueve sin que el eslabón gire, el magnetómetro queda descartado.

## 7. Prueba 5 — Registros en reposo (la medición principal)

Cierren el monitor y el plotter del Arduino IDE antes de usar Python, porque si no el puerto queda ocupado.

Instalen las dependencias una sola vez:

```
pip install pyserial numpy matplotlib
```

Busquen el puerto:

```
python bno055_etapa0.py puertos
```

Hagan **dos registros**, en modo IMU, con la IMU montada en el robot:

**Registro A — servos apagados.** Dejen la ESP32 y la IMU encendidas 5 minutos antes, para que se estabilice la temperatura. Luego:

```
python bno055_etapa0.py registrar --puerto COM5 --minutos 10 --salida reposo_servos_off.csv
```

**Registro B — servos energizados pero quietos**, sosteniendo su posición:

```
python bno055_etapa0.py registrar --puerto COM5 --minutos 10 --salida reposo_servos_on.csv
```

Durante cada registro nadie toca la mesa ni el robot. Cada 10 segundos el script muestra las muestras recibidas, las perdidas, la temperatura y el estado de calibración.

Analicen cada archivo:

```
python bno055_etapa0.py analizar reposo_servos_off.csv
python bno055_etapa0.py analizar reposo_servos_on.csv
```

Cada análisis imprime un reporte y guarda:

- `<nombre>_calibracion.json`: sesgos, varianzas R para el EKF, densidad de ruido, inestabilidad del sesgo (Allan), deriva térmica y deriva del rumbo.
- `<nombre>_series.png`, `_psd.png`, `_allan.png` y `_temperatura.png`: las gráficas.

## 8. Cómo leer los resultados

| Resultado | Qué esperar | Si sale mal |
|---|---|---|
| Frecuencia | 100 Hz | Otro programa usa el puerto, o el cable USB falla |
| Muestras perdidas y errores I²C | 0 | Revisen cables y `I2C_HZ` |
| Muestras repetidas | Pocas (menos del 2 %) | Normal: el reloj de la ESP32 y el de la BNO055 no van exactamente iguales |
| Magnitud \|a\| | Entre 9.7 y 9.9 m/s² | Recalibren el acelerómetro |
| Inclinación (roll, pitch) | Menos de 3° | La IMU no quedó plana: móntenla de nuevo o lo compensamos en el modelo |
| Media del giroscopio | Cerca de 0, menos de unos 0.1 °/s, porque la fusión ya resta el sesgo | Si es mayor, el giroscopio no estaba calibrado |
| Densidad de ruido del giroscopio | Cerca de 0.014 °/s/√Hz | Si es más de 3 veces mayor: vibración o ruido eléctrico |
| Densidad de ruido del acelerómetro | Cerca de 150 µg/√Hz | Igual que arriba |
| Deriva del rumbo en reposo (modo IMU) | Pocas décimas de grado por minuto | Si es grande, el giroscopio no está bien calibrado |

**Lo más informativo es comparar A con B.** Si el ruido de B es claramente mayor que el de A, los servos energizados están vibrando o metiendo interferencia eléctrica. Ese ruido extra es justo lo que el EKF tendrá que manejar, así que hay que saberlo antes de diseñarlo.

## 9. Lo que queda al terminar la etapa 0

- [ ] La BNO055 arranca, responde en 0x28 y la autoprueba da 0x0F
- [ ] Estado del sistema 5 y error 0
- [ ] Ejes y signos verificados, con la relación entre `gz`, el rumbo y la convención de ángulos del robot anotada
- [ ] Calibración con giroscopio y acelerómetro en 3, guardada en la ESP32 y cargada al reiniciar
- [ ] Prueba del magnetómetro cerca de los servos hecha y anotada
- [ ] Registro A (servos apagados) y registro B (servos energizados), cada uno con su JSON y sus gráficas
- [ ] Cero muestras perdidas en ambos registros

Con los dos JSON y las gráficas pasamos a la etapa 1: registrar la IMU mientras el robot hace trayectorias y compararla con lo que predice la cinemática.

## Resultados de la caracterización (27 de octubre de 2026)

Dos registros de 10 minutos en reposo, en modo IMU, a 100 Hz, con la IMU montada sobre el distal: A con los servos apagados y B con los servos energizados y quietos.

| Eje | Desv. estándar A | Desv. estándar B | Densidad de ruido B | Hoja de datos |
|---|---|---|---|---|
| gz | 0,094 °/s | 0,103 °/s | 0,0135 °/s/√Hz | 0,014 °/s/√Hz |
| gx | 0,139 °/s | 0,143 °/s | 0,010 °/s/√Hz | 0,014 °/s/√Hz |
| gy | 0,171 °/s | 0,174 °/s | 0,017 °/s/√Hz | 0,014 °/s/√Hz |
| ax | 0,042 m/s² | 0,039 m/s² | 230 µg/√Hz | 150 µg/√Hz |
| ay | 0,021 m/s² | 0,025 m/s² | 261 µg/√Hz | 150 µg/√Hz |
| az | 0,069 m/s² | 0,080 m/s² | 394 µg/√Hz | 150 µg/√Hz |

- Con los servos energizados aparecen lecturas I²C corruptas (369, 0,61 %). El firmware del robot las detecta y repite la anterior; el bus quedó en 100 kHz.
- El giroscopio está sano; el acelerómetro tiene 1,5–2,5 veces el ruido de la hoja de datos por vibración de la estructura.
- Pico de vibración en 13,2 Hz (y ~20 Hz) también con los servos apagados: no viene de los servos.
- Inclinación del montaje: pitch ≈ −4,65°.
- Valores iniciales para el EKF: R de gz = 3,2·10⁻⁶ rad²/s²; R de ax = 1,5·10⁻³ m²/s⁴; R de ay = 6,4·10⁻⁴ m²/s⁴.

**Nota:** el módulo del equipo responde en 0x29 y no arranca con cristal externo: poner `USAR_CRISTAL_EXTERNO 0` en `bno055_etapa0.ino`.

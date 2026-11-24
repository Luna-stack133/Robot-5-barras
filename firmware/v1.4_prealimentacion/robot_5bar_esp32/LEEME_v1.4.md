# Firmware 1.4: prealimentación por modelo

Es el firmware 1.3 (EKF y corrección con IMU) más una **prealimentación basada en el modelo identificado**.
Con la prealimentación apagada, los pulsos son **idénticos** a los de la 1.3 (verificado en el PC, muestra por
muestra).

## Qué hace la prealimentación

El servo sigue al comando con un tiempo muerto τ ≈ 23 ms y necesita "error" para generar par. Con el modelo
identificado el 13 nov, el firmware manda eso por adelantado:

    q_c(t) = q_plan(t + τ) + K⁻¹·[M(q)·q̈ + C(q, q̇)·q̇ + B·(1 − ff)·q̇]

- El primer término adelanta el plan el tiempo muerto del servo.
- El segundo es el par que hará falta para acelerar la inercia de esa postura (el "par calculado"),
  convertido en un desplazamiento de la referencia al dividirlo por la rigidez K del servo.
- q̇ y q̈ salen del plan por diferencias centradas de ±12 ms (suaviza los vértices de las figuras).
- Se recorta a ±10° por motor (`CTL FFMAX`). En el cuadrado 200/2000 llega a ~9°; en el pick and place a ~4°.
- Solo actúa en movimientos cartesianos (figuras y `GOTO`). No actúa en JOG ni en el cambio de modo.

Con la prealimentación encendida el brazo debe seguir el plan **sin** el atraso τ. Por eso la ley de la IMU
compara la velocidad estimada con la del plan en t (sin prealimentación la compara con t − τ, como en la 1.3).

## Comandos nuevos

```
CTL FF 0|1                 prealimentación apagada / encendida (arranca apagada)
CTL FFGAN g                escala (0..1.5; 1 = el modelo identificado)
CTL FFMAX g                recorte del desplazamiento (grados, 0..20; por defecto 10)
CTL MOD K1 K2 B1 B2 ff tau parámetros del modelo del servo (los usan el EKF y la prealimentación)
CTL                        -> CTL modo=… ff=0 ffgan=1.00 ffmax=10.0
                              CTL modelo K=1.3346/0.8651 B=0.03569/0.06636 ff=0.609 tau=0.0234
```

`CTL MOD` sirve para ajustar el modelo sin volver a cargar el firmware si, después del mantenimiento, la
identificación da otros valores. Al reiniciar la ESP32 vuelven los valores por defecto.

## Línea `I`

Agrega 6 campos al final (milésimas de grado): `q1p q2p` (el plan en el instante de la lectura), `ff1 ff2`
(prealimentación sumada a los pulsos) y `da1 da2` (corrección de la IMU realmente sumada). Con esto el análisis
ya no tiene que reconstruir el plan. La app 0.4.0 los guarda; la 0.3.x los ignora.

## Validación en el PC (`prueba_pc/lazo`)

El firmware completo, sin modificar, contra una planta con el modelo identificado. La carga se simula con
150 g más en el efector de la **planta**; el firmware no la conoce.

| Condición | Cuadrado 200/2000: contorno RMS | ±0.5 mm al parar | P&P: desviación al llegar | P&P: ±0.5 mm |
|---|---|---|---|---|
| A. Línea base | 5.84 mm | 1.10 s | 3.59 mm | 0.70 s |
| B. Prealimentación | 1.51 mm | 0.66 s | 0.92 mm | 0.34 s |
| C. IMU (α 0.3) | 4.33 mm | 0.36 s | 2.59 mm | 0.29 s |
| D. Prealimentación + IMU | 0.89 mm | 0.17 s | 0.66 mm | 0.07 s |
| A. Línea base, +150 g | 7.82 mm | 1.51 s | 4.14 mm | 0.83 s |
| B. Prealimentación, +150 g | 1.86 mm | 0.96 s | 1.12 mm | 0.47 s |
| C. IMU, +150 g | 5.95 mm | 0.66 s | 3.15 mm | 0.35 s |
| D. Prealimentación + IMU, +150 g | 1.43 mm | 0.20 s | 0.84 mm | 0.17 s |

- Con FF 0 los pulsos son idénticos a los de la 1.3.
- Con un error de ±20 % en la rigidez K del modelo, la prealimentación sigue mejorando
  (contorno 2.1 / 1.5 mm frente a 6.5 / 5.2 mm sin ella).
- **Ojo:** en esta simulación la planta es exactamente el modelo, así que la prealimentación sale casi
  perfecta. En el robot real el modelo explica solo una parte del movimiento (quedan 5–6 °/s RMS sin explicar
  en el giro del distal), así que la mejora real será menor. Justo ahí es donde se espera que la IMU aporte.

## Precauciones

- La prealimentación desplaza la referencia varios grados durante las aceleraciones. Si el driver diera una
  alarma de error de seguimiento o se oyera algo raro, bajar a `CTL FFGAN 0.5` o `CTL FFMAX 5`.
- La primera prueba se hace con ganancia 0.5, después 1.0, primero a 100/1000 y luego a 200/2000.

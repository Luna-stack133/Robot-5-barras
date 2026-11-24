#ifndef IMU_BNO055_H
#define IMU_BNO055_H
// ============================================================================
//  IMU BNO055 INTEGRADA AL FIRMWARE DEL ROBOT  (etapa 1)
//
//  - Lee la BNO055 a 100 Hz en una TAREA PROPIA, fijada al núcleo 0. Así la
//    lectura I2C (unos 5 ms a 100 kHz) nunca frena el lazo de trayectoria,
//    que corre en loop() en el núcleo 1.
//  - En el mismo instante de cada lectura guarda los ángulos de los motores
//    (los pulsos ya emitidos): IMU y cinemática quedan en el mismo reloj.
//  - Descarta las lecturas dañadas por ruido en el I2C (bytes finales en 0xFF;
//    se vieron en la etapa 0 con los servos energizados) y las repite.
//  - Usa la calibración que guardó el programa de la etapa 0 en la memoria de
//    la ESP32 (mismo espacio "bno055").
//
//  Solo la tarea de la IMU usa el bus I2C después de iniciar: no llamen a Wire
//  desde otra parte del firmware.
//
//  Firmware 1.3: después de cada lectura, la misma tarea corre el EKF y la ley
//  de corrección (control_imu.h) y agrega la estimación a la línea I.
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "control_imu.h"     // firmware 1.3: EKF + corrección en esta misma tarea

namespace imu {

// ---------------------------------------------------------------- configuración
const int      PIN_SDA    = 8;
const int      PIN_SCL    = 9;
const uint32_t I2C_HZ     = 100000;  // 100 kHz: más tolerante al ruido de los servos que 400 kHz
const uint32_t PERIODO_MS = 10;      // 100 Hz (la fusión de la BNO055 entrega datos a 100 Hz)
const int      NUCLEO     = 0;       // loop() de Arduino corre en el núcleo 1
const int      COLA       = 64;      // muestras en espera de ser enviadas (0.64 s)

// ---------------------------------------------------------------- registros
const uint8_t R_CHIP_ID = 0x00, R_PAGE_ID = 0x07, R_DATOS = 0x08, R_SYS_STATUS = 0x39;
const uint8_t R_UNIT_SEL = 0x3B, R_OPR_MODE = 0x3D, R_PWR_MODE = 0x3E, R_SYS_TRIGGER = 0x3F;
const uint8_t R_AXIS_MAP_CONFIG = 0x41, R_AXIS_MAP_SIGN = 0x42, R_OFFSETS = 0x55;
const uint8_t MODO_CONFIG = 0x00, MODO_IMU = 0x08;
const int N_DATOS = 46;              // 0x08 .. 0x35: acel, mag, gir, Euler, cuaternión, LIA, GRV, temp, calib
const int N_OFFSETS = 22;

// ---------------------------------------------------------------- muestra
struct Muestra {
    uint32_t t_us;                   // micros() al momento de la lectura
    int32_t  q1_mdeg, q2_mdeg;       // ángulos de los motores (pulsos emitidos) en milésimas de grado
    int16_t  acc[3];                 // 100 LSB = 1 m/s²
    int16_t  gir[3];                 // 16 LSB = 1 °/s
    int16_t  eul[3];                 // rumbo, roll, pitch: 16 LSB = 1°
    uint8_t  cal;                    // estado de calibración (sis, gir, acel, mag: 2 bits cada uno)
    // firmware 1.3: estimación del EKF y corrección en el instante de la lectura
    int32_t  q1h_mdeg, q2h_mdeg;     // ángulos estimados (milésimas de grado)
    int16_t  w1h, w2h;               // velocidades estimadas (centésimas de °/s)
    int16_t  d1, d2;                 // pares de perturbación (décimas de mN·m)
    int16_t  dl1, dl2;               // corrección Δq (milésimas de grado)
    int16_t  inov;                   // innovación del giroscopio (centésimas de °/s)
    uint8_t  estado;                 // bit0-1 modo (0 apagado, 1 sombra, 2 activo), bit2 EKF válido, bit3 aplicando
    // firmware 1.4: plan y lo que se sumó a los pulsos
    int32_t  q1p_mdeg, q2p_mdeg;     // plan en el instante de la lectura (milésimas de grado)
    int16_t  ff1, ff2;               // desplazamiento de la prealimentación (milésimas de grado)
    int16_t  da1, da2;               // corrección de la IMU realmente sumada (milésimas de grado)
};

// El firmware principal entrega los ángulos actuales de los motores (milésimas de grado)
typedef void (*FuncAngulos)(int32_t &q1_mdeg, int32_t &q2_mdeg);

// ---------------------------------------------------------------- estado
static uint8_t       g_addr = 0x28;
static bool          g_ok = false;           // sensor iniciado y en modo IMU
static volatile bool g_enviar = false;       // transmitir las muestras por serial (comando IMU 1)
static const char   *g_cal = "no hay";
static volatile uint32_t g_lecturas = 0, g_reintentos = 0, g_descartadas = 0, g_desbordes = 0;
static QueueHandle_t g_cola = nullptr;
static FuncAngulos   g_angulos = nullptr;

// ---------------------------------------------------------------- I2C
static bool escribir(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(g_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool leer(uint8_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(g_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint16_t)g_addr, n, true) != n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

static int leer8(uint8_t reg) {
    uint8_t v;
    return leer(reg, &v, 1) ? v : -1;
}

static int16_t i16(const uint8_t *b, int i) { return (int16_t)(b[i] | (b[i + 1] << 8)); }

static bool ponerModo(uint8_t m) {
    for (int k = 0; k < 3; k++) {
        escribir(R_OPR_MODE, MODO_CONFIG);
        delay(25);
        if (m != MODO_CONFIG) { escribir(R_OPR_MODE, m); delay(20); }
        if ((leer8(R_OPR_MODE) & 0x0F) == m) return true;
        delay(50);
    }
    return false;
}

static bool esperarChip(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (leer8(R_CHIP_ID) == 0xA0) return true;
        delay(10);
    }
    return false;
}

// ---------------------------------------------------------------- tarea de lectura
static void tarea(void *) {
    uint8_t b[N_DATOS];
    TickType_t siguiente = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&siguiente, pdMS_TO_TICKS(PERIODO_MS));
        bool ok = false;
        uint32_t t = 0;
        for (int k = 0; k < 3 && !ok; k++) {
            t = micros();
            // lectura válida: completa y sin la cola de 0xFF (temperatura -1 °C imposible)
            ok = leer(R_DATOS, b, N_DATOS) && b[44] != 0xFF;
            if (!ok) g_reintentos++;
        }
        g_lecturas++;
        if (!ok) { g_descartadas++; ctl::sinLectura(); continue; }

        Muestra m;
        m.t_us = t;
        if (g_angulos) g_angulos(m.q1_mdeg, m.q2_mdeg); else { m.q1_mdeg = 0; m.q2_mdeg = 0; }
        for (int i = 0; i < 3; i++) {
            m.acc[i] = i16(b, 0 + 2 * i);
            m.gir[i] = i16(b, 12 + 2 * i);
            m.eul[i] = i16(b, 18 + 2 * i);
        }
        m.cal = b[45];

        // ---- EKF + ley de corrección (firmware 1.3)
        {
            const float K = 0.017453293f;
            float ax = m.acc[0] / 100.0f, ay = m.acc[1] / 100.0f, az = m.acc[2] / 100.0f;
            float gz = m.gir[2] / 16.0f;
            float an = sqrtf(ax * ax + ay * ay + az * az);
            bool valida = an > 2.0f && an < 40.0f && fabsf(gz) < 500.0f;
            ctl::Salida sal;
            uint32_t t0 = micros();
            ctl::procesar(m.t_us, m.q1_mdeg / 1000.0f * K, m.q2_mdeg / 1000.0f * K, gz * K, ax, ay, valida, sal);
            ctl::anotarDuracion(micros() - t0);
            auto i16s = [](float v) -> int16_t { return (int16_t)(v > 32767.f ? 32767.f : (v < -32767.f ? -32767.f : v)); };
            m.q1h_mdeg = lroundf(sal.q1h / K * 1000.0f);
            m.q2h_mdeg = lroundf(sal.q2h / K * 1000.0f);
            m.w1h = i16s(sal.w1h / K * 100.0f);
            m.w2h = i16s(sal.w2h / K * 100.0f);
            m.d1 = i16s(sal.d1 * 10000.0f);
            m.d2 = i16s(sal.d2 * 10000.0f);
            m.dl1 = i16s(sal.dl1 / K * 1000.0f);
            m.dl2 = i16s(sal.dl2 / K * 1000.0f);
            m.inov = i16s(sal.inov / K * 100.0f);
            m.estado = sal.estado;
            m.q1p_mdeg = lroundf(sal.q1p / K * 1000.0f);
            m.q2p_mdeg = lroundf(sal.q2p / K * 1000.0f);
            m.ff1 = i16s(sal.ff1 / K * 1000.0f);
            m.ff2 = i16s(sal.ff2 / K * 1000.0f);
            m.da1 = i16s(sal.da1 / K * 1000.0f);
            m.da2 = i16s(sal.da2 / K * 1000.0f);
        }
        if (g_enviar && xQueueSend(g_cola, &m, 0) != pdTRUE) g_desbordes++;
    }
}

// ---------------------------------------------------------------- arranque
// Devuelve true si la BNO055 quedó funcionando. Si no está conectada, el robot
// sigue funcionando normal sin IMU.
inline bool iniciar(FuncAngulos angulos) {
    g_angulos = angulos;
    Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
    Wire.setTimeOut(50);
    delay(50);
    bool hay = false;
    for (uint8_t a : {(uint8_t)0x28, (uint8_t)0x29}) {
        g_addr = a;
        if (esperarChip(800)) { hay = true; break; }
    }
    if (!hay) { Serial.println("WARN imu: no se encontro la BNO055 (0x28/0x29). El robot funciona sin IMU."); return false; }

    // reinicio por software y configuración (igual que el programa de la etapa 0)
    escribir(R_OPR_MODE, MODO_CONFIG);
    delay(25);
    escribir(R_SYS_TRIGGER, 0x20);
    delay(700);
    if (!esperarChip(2000)) { Serial.println("WARN imu: la BNO055 no volvio despues del reinicio"); return false; }
    delay(50);
    escribir(R_PWR_MODE, 0x00);
    escribir(R_PAGE_ID, 0x00);
    escribir(R_UNIT_SEL, 0x00);          // m/s², °/s, grados, °C
    escribir(R_AXIS_MAP_CONFIG, 0x24);   // ejes del chip tal cual
    escribir(R_AXIS_MAP_SIGN, 0x00);
    escribir(R_SYS_TRIGGER, 0x00);       // oscilador interno (su módulo no arranca con cristal)

    // calibración guardada por el programa de la etapa 0
    Preferences prefs;
    prefs.begin("bno055", true);
    uint8_t o[N_OFFSETS];
    bool hayCal = prefs.isKey("cal") && prefs.getBytesLength("cal") == N_OFFSETS &&
                  prefs.getBytes("cal", o, N_OFFSETS) == N_OFFSETS;
    prefs.end();
    if (hayCal) {
        bool ok = true;
        for (int i = 0; i < N_OFFSETS; i++) ok &= escribir(R_OFFSETS + i, o[i]);
        g_cal = ok ? "cargada" : "error al cargar";
    }

    bool enMarcha = ponerModo(MODO_IMU);
    uint32_t t0 = millis();
    while (enMarcha && leer8(R_SYS_STATUS) != 5 && millis() - t0 < 1500) delay(20);
    if (!enMarcha || leer8(R_SYS_STATUS) != 5) {
        Serial.println("WARN imu: la BNO055 no entro en modo IMU");
        return false;
    }

    g_cola = xQueueCreate(COLA, sizeof(Muestra));
    if (!g_cola) { Serial.println("WARN imu: sin memoria para la cola"); return false; }
    xTaskCreatePinnedToCore(tarea, "imu", 12288, nullptr, 2, nullptr, NUCLEO);   // 1.3: pila mayor (EKF)
    g_ok = true;
    Serial.printf("OK imu BNO055 en 0x%02X, modo IMU, 100 Hz, calibracion %s\n", g_addr, g_cal);
    return true;
}

// ---------------------------------------------------------------- envío por serial
// Línea por muestra (enteros):  I t_us q1_mdeg q2_mdeg ax ay az gx gy gz rumbo roll pitch cal
//                               q1h q2h w1h w2h d1 d2 dl1 dl2 inov estado        (firmware 1.3)
//                               q1p q2p ff1 ff2 da1 da2                          (firmware 1.4)
// Se llama desde loop(). Envía como máximo 'maximo' muestras por llamada.
inline void enviarPendientes(int maximo = 4) {
    if (!g_ok || !g_cola) return;
    Muestra m;
    char l[320];
    for (int k = 0; k < maximo && xQueueReceive(g_cola, &m, 0) == pdTRUE; k++) {
        int n = snprintf(l, sizeof(l), "I %lu %ld %ld %d %d %d %d %d %d %d %d %d %u %ld %ld %d %d %d %d %d %d %d %u %ld %ld %d %d %d %d\n",
                         (unsigned long)m.t_us, (long)m.q1_mdeg, (long)m.q2_mdeg,
                         m.acc[0], m.acc[1], m.acc[2], m.gir[0], m.gir[1], m.gir[2],
                         m.eul[0], m.eul[1], m.eul[2], m.cal,
                         (long)m.q1h_mdeg, (long)m.q2h_mdeg, m.w1h, m.w2h, m.d1, m.d2, m.dl1, m.dl2,
                         m.inov, m.estado, (long)m.q1p_mdeg, (long)m.q2p_mdeg, m.ff1, m.ff2, m.da1, m.da2);
        Serial.write((const uint8_t *)l, n);
    }
}

inline void activarEnvio(bool on) {
    if (!on && g_cola) xQueueReset(g_cola);
    g_enviar = on;
}

inline bool enviando() { return g_ok && g_enviar; }

// Respuesta al comando "IMU" (sin argumentos)
inline void reportarEstado() {
    Serial.printf("IMU %d %d addr=0x%02X cal=%s lecturas=%lu reintentos=%lu descartadas=%lu desbordes=%lu\n",
                  g_ok ? 1 : 0, g_enviar ? 1 : 0, g_addr, g_cal, (unsigned long)g_lecturas,
                  (unsigned long)g_reintentos, (unsigned long)g_descartadas, (unsigned long)g_desbordes);
}

}  // namespace imu
#endif  // IMU_BNO055_H

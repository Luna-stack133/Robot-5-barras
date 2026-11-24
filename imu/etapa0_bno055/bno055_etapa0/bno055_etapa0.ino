// ============================================================================
//  ETAPA 0 - BNO055 en la ESP32-S3
//  Robot paralelo de 5 barras · Técnicas de Control
//
//  Objetivo: confirmar que la BNO055 responde, conocer sus modos, calibrarla,
//  guardar la calibración en la ESP32 y registrar datos para caracterizar el
//  ruido y el sesgo (eso alimenta después el EKF).
//
//  Programa independiente del firmware del robot. No usa librerías externas:
//  habla con la BNO055 directamente por registros (hoja de datos BST-BNO055-DS000).
//
//  PLACA: ESP32S3 Dev Module.  Monitor serie a 921600 baudios.
//  Si conectan por el puerto USB nativo: Tools -> USB CDC On Boot -> Enabled.
//
//  COMANDOS (una letra + Enter en el monitor serie):
//    h  ayuda                       i  modo IMU  (acel + gir, sin magnetómetro)
//    n  modo NDOF (con magnetómetro)  r  modo AMG (datos crudos, sin fusión)
//    c  estado de calibración cada 0.5 s
//    s  guardar la calibración en la ESP32     x  borrar la calibración guardada
//    o  mostrar los offsets actuales           t  repetir la autoprueba
//    a  graficar acelerómetro     g  giroscopio     e  ángulos de Euler
//    l  aceleración lineal (sin gravedad)       m  magnetómetro
//    d  datos CSV para Python (lo usa bno055_etapa0.py)
//    q  detener la salida
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

// ---------------------------------------------------------------- conexión
#define PIN_SDA  8
#define PIN_SCL  9
const uint32_t I2C_HZ = 400000;      // si hay errores de lectura, pruebe 100000
const uint32_t BAUD   = 921600;

// 1 si su módulo tiene cristal de 32.768 kHz (Adafruit y la mayoría de GY-BNO055).
// Mejora la exactitud del tiempo de la fusión. Si con 1 el sensor no arranca, déjelo en 0.
#define USAR_CRISTAL_EXTERNO 1

const uint32_t PERIODO_US = 10000;   // 100 Hz: la fusión de la BNO055 entrega datos a 100 Hz

// ---------------------------------------------------------------- registros (página 0)
const uint8_t R_CHIP_ID = 0x00, R_ACC_ID = 0x01, R_MAG_ID = 0x02, R_GYR_ID = 0x03;
const uint8_t R_SW_LSB = 0x04, R_SW_MSB = 0x05, R_BL_REV = 0x06, R_PAGE_ID = 0x07;
const uint8_t R_DATOS = 0x08;        // ACC(6) MAG(6) GIR(6) EUL(6) QUA(8) LIA(6) GRV(6) TEMP(1) CALIB(1)
const uint8_t R_CALIB_STAT = 0x35, R_ST_RESULT = 0x36, R_SYS_CLK = 0x38;
const uint8_t R_SYS_STATUS = 0x39, R_SYS_ERR = 0x3A, R_UNIT_SEL = 0x3B;
const uint8_t R_OPR_MODE = 0x3D, R_PWR_MODE = 0x3E, R_SYS_TRIGGER = 0x3F;
const uint8_t R_AXIS_MAP_CONFIG = 0x41, R_AXIS_MAP_SIGN = 0x42;
const uint8_t R_OFFSETS = 0x55;      // 22 bytes: offsets acel, mag, gir + radios acel, mag

const uint8_t MODO_CONFIG = 0x00, MODO_AMG = 0x07, MODO_IMU = 0x08, MODO_NDOF = 0x0C;
const int N_DATOS = 0x36 - R_DATOS;  // 46 bytes (0x08 .. 0x35)
const int N_OFFSETS = 22;

uint8_t g_addr = 0x28;
uint8_t g_modo = MODO_IMU;

enum Salida { SAL_NADA, SAL_DATOS, SAL_CAL, SAL_ACC, SAL_GIR, SAL_EUL, SAL_LIA, SAL_MAG };
Salida g_salida = SAL_NADA;

uint8_t  g_buf[N_DATOS];
uint32_t g_ultimoUs = 0, g_ultimoCalMs = 0;
uint32_t g_n = 0, g_errores = 0;
Preferences prefs;

// ---------------------------------------------------------------- I2C
bool escribir(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(g_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool leer(uint8_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(g_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t llegaron = Wire.requestFrom((uint16_t)g_addr, n, true);
    if (llegaron != n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

int leer8(uint8_t reg) {
    uint8_t v;
    return leer(reg, &v, 1) ? v : -1;
}

int16_t i16(const uint8_t *b, int i) { return (int16_t)(b[i] | (b[i + 1] << 8)); }

// ---------------------------------------------------------------- textos
const char *nombreModo(uint8_t m) {
    switch (m) {
        case MODO_IMU:  return "IMU (acelerometro + giroscopio, sin magnetometro)";
        case MODO_NDOF: return "NDOF (acelerometro + giroscopio + magnetometro)";
        case MODO_AMG:  return "AMG (datos crudos, sin fusion)";
        default:        return "CONFIG";
    }
}

const char *textoEstado(int s) {
    switch (s) {
        case 0: return "inactivo";
        case 1: return "ERROR del sistema";
        case 2: return "inicializando perifericos";
        case 3: return "inicializando sistema";
        case 4: return "ejecutando autoprueba";
        case 5: return "fusion en marcha";
        case 6: return "en marcha sin fusion";
        default: return "desconocido";
    }
}

const char *textoError(int e) {
    switch (e) {
        case 0: return "sin error";
        case 1: return "error de periferico";
        case 2: return "error de inicializacion";
        case 3: return "autoprueba fallida";
        case 4: return "valor de registro fuera de rango";
        case 5: return "direccion de registro fuera de rango";
        case 6: return "error al escribir un registro";
        case 7: return "modo de bajo consumo no disponible";
        case 8: return "modo de potencia del acelerometro no disponible";
        case 9: return "error de configuracion de la fusion";
        case 10: return "configuracion de sensor no disponible";
        default: return "desconocido";
    }
}

// ---------------------------------------------------------------- modos
bool ponerModo(uint8_t m) {
    if (!escribir(R_OPR_MODE, MODO_CONFIG)) return false;
    delay(25);                                   // cualquier modo -> CONFIG: 19 ms
    if (m != MODO_CONFIG) {
        if (!escribir(R_OPR_MODE, m)) return false;
        delay(20);                               // CONFIG -> otro modo: 7 ms
    }
    return true;
}

bool esperarChip(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (leer8(R_CHIP_ID) == 0xA0) return true;
        delay(10);
    }
    return false;
}

void imprimirAutoprueba(int st) {
    if (st < 0) { Serial.println("#   No se pudo leer el resultado de la autoprueba"); return; }
    Serial.printf("#   Autoprueba = 0x%02X -> acel %s, mag %s, gir %s, MCU %s\n", st,
                  (st & 1) ? "OK" : "FALLA", (st & 2) ? "OK" : "FALLA",
                  (st & 4) ? "OK" : "FALLA", (st & 8) ? "OK" : "FALLA");
}

void imprimirSistema() {
    int s = leer8(R_SYS_STATUS), e = leer8(R_SYS_ERR);
    Serial.printf("#   Estado del sistema: %d (%s) | error: %d (%s)\n", s, textoEstado(s), e, textoError(e));
}

// ---------------------------------------------------------------- calibración
bool leerOffsets(uint8_t *o) {
    uint8_t antes = g_modo;
    ponerModo(MODO_CONFIG);
    bool ok = leer(R_OFFSETS, o, N_OFFSETS);
    ponerModo(antes);
    return ok;
}

bool escribirOffsets(const uint8_t *o) {
    uint8_t antes = g_modo;
    ponerModo(MODO_CONFIG);
    bool ok = true;
    for (int i = 0; i < N_OFFSETS; i++) ok &= escribir(R_OFFSETS + i, o[i]);
    ponerModo(antes);
    return ok;
}

void imprimirOffsets(const uint8_t *o) {
    Serial.printf("#   Acel offset [mg]   x=%d y=%d z=%d   radio=%d\n", i16(o, 0), i16(o, 2), i16(o, 4), i16(o, 18));
    Serial.printf("#   Mag  offset [uT/16] x=%d y=%d z=%d   radio=%d\n", i16(o, 6), i16(o, 8), i16(o, 10), i16(o, 20));
    Serial.printf("#   Gir  offset [dps/16] x=%d y=%d z=%d\n", i16(o, 12), i16(o, 14), i16(o, 16));
}

void cargarCalibracion() {
    prefs.begin("bno055", true);
    bool hay = prefs.isKey("cal") && prefs.getBytesLength("cal") == N_OFFSETS;
    uint8_t o[N_OFFSETS];
    if (hay) prefs.getBytes("cal", o, N_OFFSETS);
    prefs.end();
    if (!hay) {
        Serial.println("# Calibracion guardada: no hay. Calibre (comando c) y guardela (comando s).");
        return;
    }
    if (escribirOffsets(o)) {
        Serial.println("# Calibracion guardada: cargada en el sensor.");
        imprimirOffsets(o);
        Serial.println("#   (el estado de calibracion puede marcar 0 hasta que el sensor vuelva a verificarla)");
    } else {
        Serial.println("# ERROR al cargar la calibracion guardada.");
    }
}

void guardarCalibracion() {
    int c = leer8(R_CALIB_STAT);
    uint8_t o[N_OFFSETS];
    if (!leerOffsets(o)) { Serial.println("ERR no se pudieron leer los offsets"); return; }
    int gir = (c >> 4) & 3, acc = (c >> 2) & 3, mag = c & 3;
    bool completa = gir == 3 && acc == 3 && (g_modo != MODO_NDOF || mag == 3);
    prefs.begin("bno055", false);
    prefs.putBytes("cal", o, N_OFFSETS);
    prefs.end();
    Serial.printf("OK calibracion guardada (gir=%d acel=%d mag=%d)\n", gir, acc, mag);
    imprimirOffsets(o);
    if (!completa) Serial.println("WARN la calibracion no estaba completa (3). Conviene repetirla y guardar de nuevo.");
}

void borrarCalibracion() {
    prefs.begin("bno055", false);
    prefs.remove("cal");
    prefs.end();
    Serial.println("OK calibracion guardada borrada (se aplica al reiniciar)");
}

void autoprueba() {
    uint8_t antes = g_modo;
    ponerModo(MODO_CONFIG);
    escribir(R_SYS_TRIGGER, 0x01 | (USAR_CRISTAL_EXTERNO ? 0x80 : 0x00));
    delay(50);
    uint32_t t0 = millis();
    while (leer8(R_SYS_STATUS) == 4 && millis() - t0 < 2000) delay(20);
    imprimirAutoprueba(leer8(R_ST_RESULT));
    ponerModo(antes);
    imprimirSistema();
}

// ---------------------------------------------------------------- arranque
bool iniciarSensor() {
    // dirección: 0x28 (ADR a GND, lo normal) o 0x29
    Serial.println("# Escaneando bus I2C...");
    bool encontrado = false;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("#   Dispositivo en 0x%02X%s\n", a, (a == 0x28 || a == 0x29) ? "  <- probablemente la BNO055" : "");
            if (!encontrado && (a == 0x28 || a == 0x29)) { g_addr = a; encontrado = true; }
        }
    }
    if (!encontrado) {
        Serial.println("# No se encontro la BNO055 en 0x28 ni 0x29. Revise VIN, GND, SDA, SCL y PS0/PS1 a GND.");
        return false;
    }
    if (!esperarChip(1000)) {
        Serial.printf("# CHIP_ID = 0x%02X (se esperaba 0xA0). No es una BNO055 o no termino de arrancar.\n", leer8(R_CHIP_ID));
        return false;
    }

    // reinicio por software: deja todo en valores de fábrica y corre la autoprueba de encendido
    escribir(R_OPR_MODE, MODO_CONFIG);
    delay(25);
    escribir(R_SYS_TRIGGER, 0x20);
    delay(700);
    if (!esperarChip(2000)) { Serial.println("# La BNO055 no volvio despues del reinicio."); return false; }
    delay(50);

    uint8_t id[7];
    leer(R_CHIP_ID, id, 7);
    Serial.printf("# CHIP_ID=0x%02X %s | ACC=0x%02X MAG=0x%02X GIR=0x%02X | firmware 0x%02X%02X | bootloader 0x%02X\n",
                  id[0], id[0] == 0xA0 ? "(correcto)" : "(INCORRECTO)", id[1], id[2], id[3], id[5], id[4], id[6]);
    imprimirAutoprueba(leer8(R_ST_RESULT));

    escribir(R_PWR_MODE, 0x00);            // potencia normal
    escribir(R_PAGE_ID, 0x00);
    // unidades: m/s2, grados/s, grados, °C; orientación de Euler tipo Windows
    escribir(R_UNIT_SEL, 0x00);
    // ejes por defecto: X, Y, Z del chip tal cual (0x24 / 0x00)
    escribir(R_AXIS_MAP_CONFIG, 0x24);
    escribir(R_AXIS_MAP_SIGN, 0x00);
    if (USAR_CRISTAL_EXTERNO) {
        escribir(R_SYS_TRIGGER, 0x80);
        delay(20);
        Serial.println("#   Reloj: cristal externo de 32.768 kHz");
    } else {
        escribir(R_SYS_TRIGGER, 0x00);
        Serial.println("#   Reloj: oscilador interno");
    }

    cargarCalibracion();
    ponerModo(g_modo);
    delay(50);
    Serial.printf("# Modo: %s\n", nombreModo(g_modo));
    imprimirSistema();
    int t = leer8(0x34);
    Serial.printf("#   Temperatura: %d C\n", (int8_t)t);
    return true;
}

void ayuda() {
    Serial.println("# Comandos (letra + Enter):");
    Serial.println("#   i modo IMU   n modo NDOF   r modo AMG (crudo)");
    Serial.println("#   c estado de calibracion   s guardar calibracion   x borrar calibracion guardada");
    Serial.println("#   o ver offsets   t autoprueba");
    Serial.println("#   a acelerometro   g giroscopio   e Euler   l acel. lineal   m magnetometro  (Serial Plotter)");
    Serial.println("#   d datos CSV para Python   q detener");
}

void setup() {
    Serial.begin(BAUD);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);
    delay(300);
    Serial.println();
    Serial.println("# ===== BNO055 · Etapa 0 · ESP32-S3 =====");
    Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
    Wire.setTimeOut(50);                    // la BNO055 estira el reloj I2C a veces
    delay(700);                             // la BNO055 tarda ~650 ms en arrancar
    if (!iniciarSensor()) {
        Serial.println("# No se pudo iniciar la BNO055. Corrija y presione RESET.");
        while (true) delay(1000);
    }
    ayuda();
    g_ultimoUs = micros();
}

// ---------------------------------------------------------------- salida
void imprimirCalibracion() {
    int c = leer8(R_CALIB_STAT);
    int s = leer8(R_SYS_STATUS), e = leer8(R_SYS_ERR);
    int t = leer8(0x34);
    if (c < 0) { Serial.println("CAL error de lectura"); return; }
    Serial.printf("CAL sistema=%d  giroscopio=%d  acelerometro=%d  magnetometro=%d | %s | estado %d error %d | T=%d C\n",
                  (c >> 6) & 3, (c >> 4) & 3, (c >> 2) & 3, c & 3,
                  g_modo == MODO_IMU ? "IMU" : g_modo == MODO_NDOF ? "NDOF" : "AMG", s, e, (int8_t)t);
}

void enviarMuestra(uint32_t t) {
    if (!leer(R_DATOS, g_buf, N_DATOS)) { g_errores++; return; }
    g_n++;
    const uint8_t *b = g_buf;
    switch (g_salida) {
        case SAL_DATOS: {
            // enteros crudos (Python los escala). Orden de registros de la BNO055.
            char l[256];
            int k = snprintf(l, sizeof(l), "D,%lu,%lu", (unsigned long)g_n, (unsigned long)t);
            for (int i = 0; i < 44; i += 2) k += snprintf(l + k, sizeof(l) - k, ",%d", i16(b, i));
            k += snprintf(l + k, sizeof(l) - k, ",%d,%u,%lu\n", (int8_t)b[44], b[45], (unsigned long)g_errores);
            Serial.write((const uint8_t *)l, k);
            break;
        }
        case SAL_ACC:
            Serial.printf("ax:%.2f,ay:%.2f,az:%.2f\n", i16(b, 0) / 100.0f, i16(b, 2) / 100.0f, i16(b, 4) / 100.0f);
            break;
        case SAL_MAG:
            Serial.printf("mx:%.1f,my:%.1f,mz:%.1f\n", i16(b, 6) / 16.0f, i16(b, 8) / 16.0f, i16(b, 10) / 16.0f);
            break;
        case SAL_GIR:
            Serial.printf("gx:%.3f,gy:%.3f,gz:%.3f\n", i16(b, 12) / 16.0f, i16(b, 14) / 16.0f, i16(b, 16) / 16.0f);
            break;
        case SAL_EUL:
            Serial.printf("rumbo:%.2f,roll:%.2f,pitch:%.2f\n", i16(b, 18) / 16.0f, i16(b, 20) / 16.0f, i16(b, 22) / 16.0f);
            break;
        case SAL_LIA:
            Serial.printf("lx:%.2f,ly:%.2f,lz:%.2f\n", i16(b, 32) / 100.0f, i16(b, 34) / 100.0f, i16(b, 36) / 100.0f);
            break;
        default: break;
    }
}

void comando(char c) {
    switch (c) {
        case 'h': ayuda(); break;
        case 'i': g_modo = MODO_IMU;  ponerModo(g_modo); Serial.printf("OK modo %s\n", nombreModo(g_modo)); break;
        case 'n': g_modo = MODO_NDOF; ponerModo(g_modo); Serial.printf("OK modo %s\n", nombreModo(g_modo)); break;
        case 'r': g_modo = MODO_AMG;  ponerModo(g_modo); Serial.printf("OK modo %s\n", nombreModo(g_modo)); break;
        case 'c': g_salida = SAL_CAL; break;
        case 's': guardarCalibracion(); break;
        case 'x': borrarCalibracion(); break;
        case 'o': { uint8_t o[N_OFFSETS]; if (leerOffsets(o)) imprimirOffsets(o); else Serial.println("ERR lectura"); break; }
        case 't': autoprueba(); break;
        case 'a': g_salida = SAL_ACC; break;
        case 'g': g_salida = SAL_GIR; break;
        case 'e': g_salida = SAL_EUL; break;
        case 'l': g_salida = SAL_LIA; break;
        case 'm': g_salida = SAL_MAG; break;
        case 'd':
            g_n = 0; g_errores = 0;
            Serial.printf("# MODO %d\n", g_modo);
            Serial.println("# D,n,t_us,ax,ay,az,mx,my,mz,gx,gy,gz,rumbo,roll,pitch,qw,qx,qy,qz,lx,ly,lz,vx,vy,vz,temp,cal,errores");
            g_salida = SAL_DATOS;
            break;
        case 'q': g_salida = SAL_NADA; Serial.println("OK detenido"); break;
        default: break;
    }
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c != '\n' && c != '\r' && c != ' ') comando(c);
    }

    if (g_salida == SAL_CAL) {
        if (millis() - g_ultimoCalMs >= 500) { g_ultimoCalMs = millis(); imprimirCalibracion(); }
        return;
    }
    if (g_salida == SAL_NADA) return;

    uint32_t ahora = micros();
    if ((uint32_t)(ahora - g_ultimoUs) >= PERIODO_US) {
        g_ultimoUs += PERIODO_US;
        if ((uint32_t)(ahora - g_ultimoUs) > PERIODO_US) g_ultimoUs = ahora;   // se atrasó mucho: resincronizar
        enviarMuestra(ahora);
    }
}

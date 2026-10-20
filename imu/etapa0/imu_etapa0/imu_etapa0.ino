/*
  =====================================================================
   Etapa 0 - Lectura de la MPU6050 con ESP32-S3
   Robot paralelo de 5 barras - Tecnicas de Control
  =====================================================================

  Que hace este programa:
    1. Busca la IMU en el bus I2C y verifica su identidad (WHO_AM_I).
    2. La configura: giroscopio +-250 deg/s, acelerometro +-2 g,
       filtro pasa bajos interno de ~44 Hz y muestreo a 200 Hz.
    3. Hace una autoprueba de 1 segundo (la IMU debe estar quieta).
    4. Queda esperando comandos por el monitor serial.

  Conexiones (modulo GY-521 -> ESP32-S3):
    VCC -> 3V3        GND -> GND
    SDA -> GPIO 8     SCL -> GPIO 9
    INT -> GPIO 10    AD0 -> GND o sin conectar (direccion 0x68)
    XDA y XCL sin conectar.

  Comandos (monitor serial a 921600 baudios, enviar una letra):
    p -> graficar acelerometro y giroscopio (Serial Plotter, 25 Hz)
    a -> graficar solo acelerometro, en g
    g -> graficar solo giroscopio, en deg/s
    c -> registro CSV a 200 Hz con datos crudos (lo usa etapa0_imu.py)
    s -> detener la salida de datos
    i -> informacion del sensor y contadores
    h -> mostrar esta ayuda

  Este programa es independiente del codigo del robot: sirve para
  verificar la IMU antes de integrarla.
*/

#include <Arduino.h>
#include <Wire.h>

// ===================== PINES (cambienlos si chocan) ======================
// Verifiquen que estos pines NO sean los que ya usan para PUL/DIR.
// Eviten: 0, 3, 45, 46 (pines de arranque), 19 y 20 (USB nativo),
// 43 y 44 (serial UART0) y 35-37 si el modulo tiene PSRAM octal (N8R8/N16R8).
static const int PIN_SDA = 8;
static const int PIN_SCL = 9;
static const int PIN_INT = 10;   // si INT no esta conectado, se usa sondeo por I2C

// ===================== CONFIGURACION =====================================
static const uint32_t BAUDIOS  = 921600;
static const uint32_t I2C_HZ   = 400000;
static const uint8_t  MPU_ADDR = 0x68;

// Filtro pasa bajos interno (DLPF) y frecuencia de muestreo:
//   DLPF_CFG = 3   -> acelerometro ~44 Hz, giroscopio ~42 Hz (salida interna 1 kHz)
//   SMPLRT_DIV = 4 -> 1000 / (1 + 4) = 200 Hz
static const uint8_t DLPF_CFG   = 3;
static const uint8_t SMPLRT_DIV = 4;
static const float   FS_HZ      = 1000.0f / (1 + SMPLRT_DIV);

// Rangos: giroscopio +-250 deg/s (131 LSB por deg/s), acelerometro +-2 g (16384 LSB por g)
static const float GIRO_LSB_DPS = 131.0f;
static const float ACEL_LSB_G   = 16384.0f;

// En los modos de grafica se imprime 1 de cada 8 muestras (200 / 8 = 25 Hz)
static const uint16_t DIEZMADO_PLOTTER = 8;

// ===================== REGISTROS DE LA MPU6050 ===========================
static const uint8_t REG_SMPLRT_DIV    = 0x19;
static const uint8_t REG_CONFIG        = 0x1A;
static const uint8_t REG_GYRO_CONFIG   = 0x1B;
static const uint8_t REG_ACCEL_CONFIG  = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2 = 0x1D;   // solo existe en la familia MPU6500
static const uint8_t REG_INT_PIN_CFG   = 0x37;
static const uint8_t REG_INT_ENABLE    = 0x38;
static const uint8_t REG_INT_STATUS    = 0x3A;
static const uint8_t REG_ACCEL_XOUT_H  = 0x3B;
static const uint8_t REG_PWR_MGMT_1    = 0x6B;
static const uint8_t REG_WHO_AM_I      = 0x75;

// ===================== ESTADO ============================================
enum Modo { PARADO, CSV, PLOT_TODO, PLOT_ACEL, PLOT_GIRO };
static Modo modo = PARADO;

struct Muestra {
  uint32_t t_us;
  int16_t ax, ay, az, temp, gx, gy, gz;
};

static uint8_t  whoAmI      = 0;
static bool     familia6500 = false;   // WHO_AM_I 0x70/0x71/0x73: chip de la familia MPU6500
static bool     usarINT     = false;
static uint32_t nMuestras   = 0;
static uint32_t erroresI2C  = 0;
static uint32_t ultimoSondeo = 0;

// Variables compartidas con la interrupcion
static volatile bool     hayDato  = false;
static volatile uint32_t tDato_us = 0;
static volatile uint32_t perdidas = 0;   // muestras que llegaron antes de leer la anterior
static volatile uint32_t nIsr     = 0;

void IRAM_ATTR isrDatoListo() {
  if (hayDato) perdidas = perdidas + 1;
  tDato_us = micros();
  hayDato = true;
  nIsr = nIsr + 1;
}

// ===================== I2C ===============================================
bool escribirReg(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

bool leerRegs(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)MPU_ADDR, (size_t)n, true) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool leerReg(uint8_t reg, uint8_t &valor) {
  return leerRegs(reg, &valor, 1);
}

void escanearI2C() {
  Serial.println(F("# Escaneando bus I2C..."));
  int encontrados = 0;
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) {
      Serial.printf("#   Dispositivo en 0x%02X%s\n", dir,
                    (dir == 0x68 || dir == 0x69) ? "  <- probablemente la IMU" : "");
      encontrados++;
    }
  }
  if (encontrados == 0) {
    Serial.println(F("#   Ninguno. Revisen VCC, GND, SDA y SCL (y que no esten invertidos)."));
  }
}

// ===================== CONFIGURACION DE LA IMU ===========================
bool iniciarMPU() {
  if (!escribirReg(REG_PWR_MGMT_1, 0x80)) return false;   // reinicio del chip
  delay(100);
  if (!escribirReg(REG_PWR_MGMT_1, 0x01)) return false;   // despertar, reloj del giroscopio X
  delay(50);

  if (!leerReg(REG_WHO_AM_I, whoAmI)) return false;
  familia6500 = (whoAmI == 0x70 || whoAmI == 0x71 || whoAmI == 0x73);

  bool ok = true;
  ok &= escribirReg(REG_SMPLRT_DIV, SMPLRT_DIV);
  ok &= escribirReg(REG_CONFIG, DLPF_CFG);
  ok &= escribirReg(REG_GYRO_CONFIG, 0x00);    // +-250 deg/s
  ok &= escribirReg(REG_ACCEL_CONFIG, 0x00);   // +-2 g
  if (familia6500) {
    ok &= escribirReg(REG_ACCEL_CONFIG2, 0x03);  // filtro del acelerometro ~45 Hz (MPU6500)
  }
  // INT: activo en alto, push-pull, pulso de 50 us, se limpia con cualquier lectura
  ok &= escribirReg(REG_INT_PIN_CFG, 0x10);
  ok &= escribirReg(REG_INT_ENABLE, 0x01);     // interrupcion de "dato listo"
  return ok;
}

void reportarIdentidad() {
  Serial.printf("# WHO_AM_I = 0x%02X -> ", whoAmI);
  if (whoAmI == 0x68) {
    Serial.println(F("MPU6050 (correcto)"));
  } else if (familia6500) {
    Serial.println(F("chip de la familia MPU6500/9250 (no es una MPU6050 original)."));
    Serial.println(F("#   Los datos de acelerometro y giroscopio son compatibles; se continua."));
  } else {
    Serial.println(F("valor inesperado: probablemente un clon."));
    Serial.println(F("#   Se continua, pero verifiquen los datos con cuidado."));
  }
}

// ===================== LECTURA ===========================================
bool leerMuestra(Muestra &m) {
  uint8_t b[14];
  if (!leerRegs(REG_ACCEL_XOUT_H, b, 14)) return false;
  m.ax   = (int16_t)((b[0]  << 8) | b[1]);
  m.ay   = (int16_t)((b[2]  << 8) | b[3]);
  m.az   = (int16_t)((b[4]  << 8) | b[5]);
  m.temp = (int16_t)((b[6]  << 8) | b[7]);
  m.gx   = (int16_t)((b[8]  << 8) | b[9]);
  m.gy   = (int16_t)((b[10] << 8) | b[11]);
  m.gz   = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

float temperaturaC(int16_t crudo) {
  return familia6500 ? crudo / 333.87f + 21.0f : crudo / 340.0f + 36.53f;
}

// Devuelve true cuando hay una muestra nueva y entrega su marca de tiempo.
bool hayMuestraNueva(uint32_t &t) {
  if (usarINT) {
    if (!hayDato) return false;
    noInterrupts();
    t = tDato_us;
    hayDato = false;
    interrupts();
    return true;
  }
  // Sin INT: se consulta el registro de estado cada 500 us
  uint32_t ahora = micros();
  if (ahora - ultimoSondeo < 500) return false;
  ultimoSondeo = ahora;
  uint8_t estado;
  if (!leerReg(REG_INT_STATUS, estado)) { erroresI2C++; return false; }
  if (!(estado & 0x01)) return false;
  t = ahora;
  return true;
}

// ===================== AUTOPRUEBA ========================================
void autoprueba() {
  Serial.println(F("# Autoprueba: mantengan la IMU quieta durante 1 segundo..."));
  const int N = (int)FS_HZ;
  double s[6] = {0, 0, 0, 0, 0, 0};
  int n = 0;
  uint32_t inicio = millis();
  while (n < N && millis() - inicio < 3000) {
    uint32_t t;
    if (!hayMuestraNueva(t)) continue;
    Muestra m;
    if (!leerMuestra(m)) { erroresI2C++; continue; }
    s[0] += m.ax / ACEL_LSB_G;   s[1] += m.ay / ACEL_LSB_G;   s[2] += m.az / ACEL_LSB_G;
    s[3] += m.gx / GIRO_LSB_DPS; s[4] += m.gy / GIRO_LSB_DPS; s[5] += m.gz / GIRO_LSB_DPS;
    n++;
  }
  if (n == 0) {
    Serial.println(F("#   No llegaron muestras. Revisen la conexion."));
    return;
  }
  for (int i = 0; i < 6; i++) s[i] /= n;
  float norma = sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
  Serial.printf("#   Muestras: %d en %.2f s\n", n, (millis() - inicio) / 1000.0f);
  Serial.printf("#   Acelerometro medio [g]:   x=%+.3f  y=%+.3f  z=%+.3f\n", s[0], s[1], s[2]);
  Serial.printf("#   Magnitud |a| = %.3f g  %s\n", norma,
                (norma > 0.9f && norma < 1.1f) ? "(bien, debe ser cercana a 1)" : "(REVISAR: deberia ser cercana a 1)");
  Serial.printf("#   Giroscopio medio [deg/s]: x=%+.2f  y=%+.2f  z=%+.2f  (sesgo en reposo)\n", s[3], s[4], s[5]);
}

// ===================== SALIDA DE DATOS ===================================
void emitir(const Muestra &m) {
  switch (modo) {
    case CSV:
      Serial.printf("%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%lu,%lu\n",
                    (unsigned long)nMuestras, (unsigned long)m.t_us,
                    m.ax, m.ay, m.az, m.temp, m.gx, m.gy, m.gz,
                    (unsigned long)perdidas, (unsigned long)erroresI2C);
      break;
    case PLOT_TODO:
      if (nMuestras % DIEZMADO_PLOTTER) break;
      Serial.printf("ax:%.3f,ay:%.3f,az:%.3f,gx:%.2f,gy:%.2f,gz:%.2f\n",
                    m.ax / ACEL_LSB_G, m.ay / ACEL_LSB_G, m.az / ACEL_LSB_G,
                    m.gx / GIRO_LSB_DPS, m.gy / GIRO_LSB_DPS, m.gz / GIRO_LSB_DPS);
      break;
    case PLOT_ACEL:
      if (nMuestras % DIEZMADO_PLOTTER) break;
      Serial.printf("ax_g:%.3f,ay_g:%.3f,az_g:%.3f\n",
                    m.ax / ACEL_LSB_G, m.ay / ACEL_LSB_G, m.az / ACEL_LSB_G);
      break;
    case PLOT_GIRO:
      if (nMuestras % DIEZMADO_PLOTTER) break;
      Serial.printf("gx_dps:%.2f,gy_dps:%.2f,gz_dps:%.2f\n",
                    m.gx / GIRO_LSB_DPS, m.gy / GIRO_LSB_DPS, m.gz / GIRO_LSB_DPS);
      break;
    default:
      break;
  }
}

void imprimirAyuda() {
  Serial.println(F("# Comandos: p=graficar todo  a=acelerometro  g=giroscopio"));
  Serial.println(F("#           c=registro CSV  s=detener  i=informacion  h=ayuda"));
}

void imprimirInfo() {
  Serial.printf("# WHO_AM_I: 0x%02X | Muestreo: %.0f Hz | DLPF_CFG: %u | Lectura: %s\n",
                whoAmI, FS_HZ, DLPF_CFG, usarINT ? "interrupcion INT" : "sondeo (INT no conectado)");
  Serial.printf("# Pines: SDA=%d SCL=%d INT=%d | I2C: %lu Hz\n",
                PIN_SDA, PIN_SCL, PIN_INT, (unsigned long)I2C_HZ);
  Serial.printf("# Muestras: %lu | Perdidas: %lu | Errores I2C: %lu\n",
                (unsigned long)nMuestras, (unsigned long)perdidas, (unsigned long)erroresI2C);
  Muestra m;
  if (leerMuestra(m)) Serial.printf("# Temperatura del sensor: %.1f C\n", temperaturaC(m.temp));
}

void iniciarCSV() {
  noInterrupts();
  hayDato = false;
  perdidas = 0;
  interrupts();
  nMuestras = 0;
  erroresI2C = 0;
  Serial.printf("# cfg fs_hz=%.1f dlpf_cfg=%u giro_lsb_dps=%.1f acel_lsb_g=%.1f who_am_i=0x%02X lectura=%s\n",
                FS_HZ, DLPF_CFG, GIRO_LSB_DPS, ACEL_LSB_G, whoAmI, usarINT ? "int" : "sondeo");
  Serial.println(F("n,t_us,ax,ay,az,temp,gx,gy,gz,perdidas,err_i2c"));
  modo = CSV;
}

void atenderComandos() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'p': modo = PLOT_TODO; break;
      case 'a': modo = PLOT_ACEL; break;
      case 'g': modo = PLOT_GIRO; break;
      case 'c': iniciarCSV(); break;
      case 's': modo = PARADO; Serial.println(F("# Detenido.")); break;
      case 'i': imprimirInfo(); break;
      case 'h': case '?': imprimirAyuda(); break;
      default: break;   // se ignoran saltos de linea y otros caracteres
    }
  }
}

// ===================== SETUP Y LOOP ======================================
void setup() {
  Serial.begin(BAUDIOS);
  delay(1500);   // tiempo para abrir el monitor serial
  Serial.println();
  Serial.println(F("# ===== Etapa 0: MPU6050 + ESP32-S3 ====="));

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  escanearI2C();

  if (!iniciarMPU()) {
    Serial.println(F("# ERROR: la IMU no responde en 0x68. Revisen conexiones y el pin AD0."));
    Serial.println(F("#        El programa queda detenido."));
    while (true) delay(1000);
  }
  reportarIdentidad();

  // Detectar si el pin INT esta conectado: a 200 Hz deben llegar ~40 pulsos en 200 ms
  pinMode(PIN_INT, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), isrDatoListo, RISING);
  nIsr = 0;
  delay(200);
  uint32_t pulsos = nIsr;
  usarINT = (pulsos >= 20 && pulsos <= 80);
  if (usarINT) {
    Serial.printf("# INT conectado en GPIO %d (%lu pulsos en 200 ms).\n", PIN_INT, (unsigned long)pulsos);
  } else {
    detachInterrupt(digitalPinToInterrupt(PIN_INT));
    Serial.printf("# AVISO: no se detectaron pulsos en GPIO %d. Se usara sondeo por I2C.\n", PIN_INT);
    Serial.println(F("#        Funciona, pero las marcas de tiempo tienen hasta 0.5 ms de error."));
  }
  noInterrupts();
  hayDato = false;
  perdidas = 0;
  interrupts();

  autoprueba();
  imprimirAyuda();
}

void loop() {
  atenderComandos();

  uint32_t t;
  if (!hayMuestraNueva(t)) return;

  Muestra m;
  if (!leerMuestra(m)) { erroresI2C++; return; }
  m.t_us = t;
  nMuestras++;
  emitir(m);
}

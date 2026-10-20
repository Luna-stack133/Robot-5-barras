// ============================================================================
//  ROBOT PARALELO 5 BARRAS  -  FIRMWARE ESP32-S3
//  Servos integrados iHSV57 (Step/Dir).  El servo cierra su propio lazo;
//  este firmware genera trayectoria + pasos coordinados y la seguridad.
//
//  DEPENDENCIAS (Library Manager de Arduino):
//    - FastAccelStepper   (gin66)
//  PLACA:  ESP32-S3 (Arduino-ESP32 core).  Serial = USB CDC nativo.
//
//  Coloca kinematics_5bar.h en la MISMA carpeta que este .ino.
//
//  PROTOCOLO SERIE (lineas de texto, terminadas en '\n'), 115200 bps:
//    Host -> ESP32:
//      PING                      -> "OK PONG"
//      GEOM                      -> "GEOM L1 L2 L3 L4 D" (para sincronizar la GUI)
//      JOG <eje 1|2> <grados>    nudge manual de un eje (para colocar el utillaje)
//      SETHOME [th1 th2]         fija el cero con el utillaje puesto (grados; sin
//                                argumentos usa TH1_HOME_DEG/TH2_HOME_DEG)
//      SETV <vmax> <amax>        limites de vel/acel del efector (m/s, m/s2)
//      GOTO <x> <y>              mover a un punto cartesiano (m)
//      PATH <n> x1 y1 x2 y2 ...  recorrer n puntos. Cerrar = repetir 1er punto
//      STOP                      paro inmediato (aborta movimiento)
//      STATUS                    pide un STATE inmediato
//    ESP32 -> Host:
//      OK <msg> / ERR <msg>
//      EVT HOMED | DONE | LIMIT
//      STATE <estado> <x> <y> <th1deg> <th2deg> <moving>   (telemetria ~20 Hz)
// ============================================================================
#include <Arduino.h>
#include <FastAccelStepper.h>
#include "kinematics_5bar.h"
using namespace rk;

// ============================================================================
//  CONFIGURACION DE PINES  (solo PUL/DIR -- sin ENABLE por ahora)
//  Evita pines strapping/USB del ESP32-S3.
//
//  CABLEADO (tu foto: comun-anodo, manual iHSV57 cap. 6.1):
//    PUL+ , DIR+  de AMBOS servos  -> +5 V   (anodo comun; a 5V sin resistencia)
//    PUL- , DIR-  servo 1 y 2      -> salidas del ESP32 (via buffer, ver abajo)
//    GND servos  <->  GND ESP32
//
//  POLARIDAD CON ANODO COMUN (importante):
//  En anodo comun el opto se ENCIENDE al poner el catodo (PUL-/DIR-) a nivel
//  BAJO. Lo correcto es que el LED del opto este apagado en reposo y solo
//  parpadee en cada paso -> el pulso debe ser ACTIVO-BAJO en el opto.
//
//  FastAccelStepper genera pulsos ACTIVO-ALTO (3.3 V). Por eso, entre el ESP32
//  y PUL-/DIR- conviene un BUFFER INVERSOR a 5 V:  74HCT14  (hex Schmitt
//  inversor; el tipo HCT reconoce 3.3 V como alto). Resuelve 3 cosas a la vez:
//    1) nivel 5 V (apaga bien el opto, que a solo 3.3 V quedaria a medias),
//    2) inversion -> pulso activo-bajo (LEDs del opto apagados en reposo),
//    3) flancos limpios (Schmitt).
//  Pasa los 4 hilos (PUL1/DIR1/PUL2/DIR2) por el 74HCT14. La inversion tambien
//  afecta a DIR (solo invierte el sentido): si un motor gira al reves, pon su
//  DIR_SIGN en -1 mas abajo.  (Un buffer NO inversor -74HCT245- tambien cuenta
//  pasos, pero deja los LEDs del opto encendidos en reposo: calor inutil.)
// ============================================================================
#define PIN_PUL1   4     // STEP eje 1  -> PUL- servo 1
#define PIN_DIR1   5     // DIR  eje 1  -> DIR- servo 1
#define PIN_PUL2   6     // STEP eje 2  -> PUL- servo 2
#define PIN_DIR2   7     // DIR  eje 2  -> DIR- servo 2

// Retardo de cambio de DIR antes de pulsar (us). Los optos son lentos: deja que
// la linea DIR se estabilice a traves del opto antes del flanco de STEP.
const uint16_t DIR_CHANGE_DELAY_US = 40;

// ============================================================================
//  SIGNOS DE DIRECCION (+1 / -1). Cambia si un motor gira al reves.
// ============================================================================
const int DIR_SIGN1 = +1;
const int DIR_SIGN2 = +1;

// ============================================================================
//  CERO / HOMING
//  Metodo por defecto: UTILLAJE de referencia (pieza impresa que bloquea los
//  brazos en una pose unica).  TH1_HOME_DEG / TH2_HOME_DEG son los angulos
//  EXACTOS en que el utillaje deja cada brazo (mide/calcula los de TU pieza).
//  Deben dar un ensamblaje valido de IK config 3.
//
//  Procedimiento (ver comando SETHOME):
//    1) encajar el utillaje (los brazos quedan en la pose de referencia)
//    2) SETHOME    (fija el cero a TH1_HOME_DEG/TH2_HOME_DEG; NO mueve)
//    3) retirar el utillaje -> listo
//  Usa JOG para acercar los brazos al utillaje o verificar el sentido de giro.
// ============================================================================
const float TH1_HOME_DEG = 90.0f;   // <-- angulo del brazo izq. con el utillaje puesto
const float TH2_HOME_DEG = 90.0f;   // <-- angulo del brazo der. con el utillaje puesto
const float JOG_SPEED_HZ = 600.0f;  // pasos/s durante jog (lento y seguro)

// ============================================================================
//  LIMITES BLANDOS DE ARTICULACION (grados). Detiene si se exceden.
// ============================================================================
const float TH1_MIN_DEG = 10.0f,  TH1_MAX_DEG = 250.0f;
const float TH2_MIN_DEG = -70.0f, TH2_MAX_DEG = 250.0f;

// ============================================================================
//  PARAMETROS DE MOVIMIENTO Y EJECUCION
// ============================================================================
float    g_Vmax = 0.10f;     // m/s   (modificable por SETV; arranca conservador)
float    g_Amax = 0.20f;     // m/s2
const uint32_t TICK_US   = 4000;     // 250 Hz: actualizacion del setpoint
const float    MAX_HZ    = 60000.0f; // tope de FastAccelStepper (no debe limitar)
const float    MAX_ACC   = 400000.0f;
const uint32_t TELEM_MS  = 50;       // 20 Hz de telemetria

#define MAX_WP 130
float    g_wx[MAX_WP], g_wy[MAX_WP];
int      g_nWp = 0;

// ============================================================================
//  ESTADO
// ============================================================================
enum State { ST_INIT, ST_IDLE, ST_JOGGING, ST_MOVING, ST_HALTED };
State g_state = ST_INIT;
bool  g_homed = false;

float g_curX = 0, g_curY = 0;   // posicion cartesiana comandada actual

// segmento en ejecucion
int      g_seg = 0;
float    g_segP0x, g_segP0y, g_segP1x, g_segP1y, g_segT;
uint32_t g_segStartUs = 0;
uint32_t g_lastTickUs = 0;
uint32_t g_lastTelemMs = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *st1 = nullptr;
FastAccelStepper *st2 = nullptr;

char  g_line[4200];
int   g_lineLen = 0;

// ---------------------------------------------------------------------------
long stepsAxis1(float th) { return lroundf(DIR_SIGN1 * th * STEPS_PER_RAD); }
long stepsAxis2(float th) { return lroundf(DIR_SIGN2 * th * STEPS_PER_RAD); }

void sendState() {
    float t1 = (st1 ? st1->getCurrentPosition() : 0) / (DIR_SIGN1 * STEPS_PER_RAD);
    float t2 = (st2 ? st2->getCurrentPosition() : 0) / (DIR_SIGN2 * STEPS_PER_RAD);
    bool moving = (g_state == ST_MOVING) || (g_state == ST_JOGGING);
    const char* s = (g_state==ST_IDLE)?"IDLE":(g_state==ST_JOGGING)?"JOGGING":
                    (g_state==ST_MOVING)?"MOVING":(g_state==ST_HALTED)?"HALTED":"INIT";
    Serial.printf("STATE %s %.4f %.4f %.2f %.2f %d\n",
                  s, g_curX, g_curY, degrees(t1), degrees(t2), moving ? 1 : 0);
}

bool jointLimitsOK(float th1, float th2) {
    float d1 = degrees(th1), d2 = degrees(th2);
    return d1 >= TH1_MIN_DEG && d1 <= TH1_MAX_DEG &&
           d2 >= TH2_MIN_DEG && d2 <= TH2_MAX_DEG;
}

void haltAll(const char* reason) {
    if (st1) st1->forceStop();
    if (st2) st2->forceStop();
    g_state = ST_HALTED;
    g_nWp = 0;
    Serial.printf("EVT %s\n", reason);
}

// Mueve ambos ejes al punto (x,y) si es seguro. false si no.
bool commandCartesian(float x, float y) {
    float t1, t2;
    if (!ik5bar(x, y, t1, t2)) return false;
    if (singularityMeasure(x, y, t1, t2) < SIN_MIN) return false;
    if (!jointLimitsOK(t1, t2)) return false;
    st1->moveTo(stepsAxis1(t1));
    st2->moveTo(stepsAxis2(t2));
    g_curX = x; g_curY = y;
    return true;
}

bool validatePath() {
    for (int i = 0; i < g_nWp; i++) {
        if (!isReachableAndSafe(g_wx[i], g_wy[i])) {
            Serial.printf("ERR punto %d (%.3f,%.3f) inalcanzable o singular\n",
                          i, g_wx[i], g_wy[i]);
            return false;
        }
    }
    return true;
}

void enterSegment(int seg) {
    g_seg = seg;
    g_segP0x = g_wx[seg];   g_segP0y = g_wy[seg];
    g_segP1x = g_wx[seg+1]; g_segP1y = g_wy[seg+1];
    float dx = g_segP1x - g_segP0x, dy = g_segP1y - g_segP0y;
    float dist = sqrtf(dx*dx + dy*dy);
    g_segT = segDuration(dist, g_Vmax, g_Amax);
    g_segStartUs = micros();
}

void startPath() {
    if (!g_homed)   { Serial.println("ERR no homed"); return; }
    if (g_nWp < 2)  { Serial.println("ERR path needs >= 2 points"); return; }
    if (!validatePath()) return;
    st1->setSpeedInHz((uint32_t)MAX_HZ);   // por si jog/home dejaron vel baja
    st2->setSpeedInHz((uint32_t)MAX_HZ);
    enterSegment(0);
    g_state = ST_MOVING;
    g_lastTickUs = micros();
    Serial.printf("OK moving %d segments\n", g_nWp - 1);
}

void updateMoving() {
    uint32_t now = micros();
    if ((uint32_t)(now - g_lastTickUs) < TICK_US) return;
    g_lastTickUs = now;

    float elapsed = (now - g_segStartUs) * 1e-6f;
    float tau = (g_segT > 0) ? (elapsed / g_segT) : 1.0f;

    if (tau >= 1.0f) {
        if (!commandCartesian(g_segP1x, g_segP1y)) { haltAll("LIMIT"); return; }
        if (g_seg + 1 < g_nWp - 1) {
            enterSegment(g_seg + 1);
        } else {
            g_state = ST_IDLE;
            Serial.println("EVT DONE");
        }
        return;
    }

    float s = quinticS(tau);
    float x = g_segP0x + (g_segP1x - g_segP0x) * s;
    float y = g_segP0y + (g_segP1y - g_segP0y) * s;
    if (!commandCartesian(x, y)) { haltAll("LIMIT"); return; }
}

// ---------------------------------------------------------------------------
//  CERO POR UTILLAJE: fija la posicion actual a (th1deg, th2deg).
//  Con el utillaje puesto y los servos habilitados, redefine el contador de
//  pasos: a partir de aqui el robot sabe su angulo absoluto.  No mueve nada.
// ---------------------------------------------------------------------------
void setHome(float th1deg, float th2deg) {
    float t1 = radians(th1deg), t2 = radians(th2deg);
    st1->setCurrentPosition(stepsAxis1(t1));
    st2->setCurrentPosition(stepsAxis2(t2));
    fk5bar(t1, t2, g_curX, g_curY);
    g_homed = true;
    if (g_state == ST_HALTED || g_state == ST_INIT) g_state = ST_IDLE;
    Serial.printf("OK home set th1=%.2f th2=%.2f -> x=%.4f y=%.4f\n",
                  th1deg, th2deg, g_curX, g_curY);
    Serial.println("EVT HOMED");
}

// ---------------------------------------------------------------------------
//  JOG: nudge manual de un eje (grados, relativo) a velocidad baja.
//  Sirve para acercar los brazos al utillaje o verificar el sentido de giro.
//  Se permite aun sin cero fijado (precisamente para colocar el utillaje).
// ---------------------------------------------------------------------------
void jogAxis(int axis, float deltadeg) {
    if (g_state == ST_MOVING) { Serial.println("ERR busy"); return; }
    long dsteps = lroundf(radians(fabsf(deltadeg)) * STEPS_PER_RAD);
    if (dsteps == 0) return;
    FastAccelStepper* st = (axis == 1) ? st1 : st2;
    int sign = (axis == 1) ? DIR_SIGN1 : DIR_SIGN2;
    st->setSpeedInHz((uint32_t)JOG_SPEED_HZ);
    st->move(sign * (deltadeg >= 0 ? dsteps : -dsteps));
    g_state = ST_JOGGING;
    Serial.printf("OK jog axis %d %.2f deg\n", axis, deltadeg);
}


void handleLine(char* line) {
    char* tok = strtok(line, " \t\r\n");
    if (!tok) return;

    if (!strcmp(tok, "PING"))   { Serial.println("OK PONG"); return; }
    if (!strcmp(tok, "GEOM")) {   // reporta la geometria para que la GUI se sincronice
        Serial.printf("GEOM %.4f %.4f %.4f %.4f %.4f\n", L1, L2, L3, L4, D);
        return;
    }
    if (!strcmp(tok, "STATUS")) { sendState(); return; }

    if (!strcmp(tok, "STOP")) {
        if (st1) st1->forceStop();
        if (st2) st2->forceStop();
        g_nWp = 0;
        g_state = g_homed ? ST_IDLE : ST_INIT;
        Serial.println("OK stopped");
        return;
    }

    // CERO POR UTILLAJE.  "SETHOME"            -> usa TH1_HOME_DEG/TH2_HOME_DEG
    //                     "SETHOME th1 th2"    -> angulos explicitos (grados)
    if (!strcmp(tok, "SETHOME")) {
        char* a = strtok(NULL, " \t\r\n");
        char* b = strtok(NULL, " \t\r\n");
        float th1 = a ? atof(a) : TH1_HOME_DEG;
        float th2 = b ? atof(b) : TH2_HOME_DEG;
        setHome(th1, th2);
        return;
    }

    // JOG axis(1|2) delta_grados   (nudge manual a velocidad baja)
    if (!strcmp(tok, "JOG")) {
        char* a = strtok(NULL, " \t\r\n");
        char* b = strtok(NULL, " \t\r\n");
        if (!a || !b) { Serial.println("ERR jog <eje 1|2> <grados>"); return; }
        int ax = atoi(a);
        if (ax != 1 && ax != 2) { Serial.println("ERR eje 1 o 2"); return; }
        jogAxis(ax, atof(b));
        return;
    }

    if (!strcmp(tok, "SETV")) {
        char* a = strtok(NULL, " \t\r\n");
        char* b = strtok(NULL, " \t\r\n");
        if (a) g_Vmax = atof(a);
        if (b) g_Amax = atof(b);
        if (g_Vmax < 0.01f) g_Vmax = 0.01f;
        if (g_Amax < 0.01f) g_Amax = 0.01f;
        Serial.printf("OK setv %.3f %.3f\n", g_Vmax, g_Amax);
        return;
    }

    if (!strcmp(tok, "GOTO")) {
        char* a = strtok(NULL, " \t\r\n");
        char* b = strtok(NULL, " \t\r\n");
        if (!a || !b) { Serial.println("ERR goto x y"); return; }
        g_wx[0] = g_curX; g_wy[0] = g_curY;
        g_wx[1] = atof(a); g_wy[1] = atof(b);
        g_nWp = 2;
        startPath();
        return;
    }

    if (!strcmp(tok, "PATH")) {
        char* a = strtok(NULL, " \t\r\n");
        if (!a) { Serial.println("ERR path n ..."); return; }
        int n = atoi(a);
        if (n < 2 || n > MAX_WP) { Serial.printf("ERR n out of range (2..%d)\n", MAX_WP); return; }
        int i = 0;
        for (; i < n; i++) {
            char* xs = strtok(NULL, " \t\r\n");
            char* ys = strtok(NULL, " \t\r\n");
            if (!xs || !ys) break;
            g_wx[i] = atof(xs); g_wy[i] = atof(ys);
        }
        if (i != n) { Serial.println("ERR path point count mismatch"); return; }
        g_nWp = n;
        startPath();
        return;
    }

    Serial.printf("ERR unknown cmd: %s\n", tok);
}

void pollSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            g_line[g_lineLen] = 0;
            handleLine(g_line);
            g_lineLen = 0;
        } else if (g_lineLen < (int)sizeof(g_line) - 1) {
            g_line[g_lineLen++] = c;
        } else {
            g_lineLen = 0;  // linea demasiado larga: descartar
        }
    }
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);

    engine.init();
    st1 = engine.stepperConnectToPin(PIN_PUL1);
    st2 = engine.stepperConnectToPin(PIN_PUL2);
    if (!st1 || !st2) { Serial.println("ERR stepper init"); while (1) delay(1000); }

    // DIR con retardo de cambio (deja estabilizar el opto antes del pulso)
    st1->setDirectionPin(PIN_DIR1, true, DIR_CHANGE_DELAY_US);
    st2->setDirectionPin(PIN_DIR2, true, DIR_CHANGE_DELAY_US);
    st1->setAcceleration((uint32_t)MAX_ACC);
    st2->setAcceleration((uint32_t)MAX_ACC);
    st1->setSpeedInHz((uint32_t)MAX_HZ);
    st2->setSpeedInHz((uint32_t)MAX_HZ);

    g_state = ST_INIT;
    Serial.println("OK ready - 5bar ESP32-S3");
    Serial.printf("OK PPR=%.0f Vmax=%.3f Amax=%.3f\n", STEPS_PER_REV, g_Vmax, g_Amax);
}

void loop() {
    pollSerial();

    switch (g_state) {
        case ST_MOVING: updateMoving(); break;
        case ST_JOGGING:
            if (!st1->isRunning() && !st2->isRunning()) {
                st1->setSpeedInHz((uint32_t)MAX_HZ);
                st2->setSpeedInHz((uint32_t)MAX_HZ);
                g_state = g_homed ? ST_IDLE : ST_INIT;
            }
            break;
        default: break;
    }

    uint32_t ms = millis();
    if ((uint32_t)(ms - g_lastTelemMs) >= TELEM_MS) {
        g_lastTelemMs = ms;
        sendState();
    }
}

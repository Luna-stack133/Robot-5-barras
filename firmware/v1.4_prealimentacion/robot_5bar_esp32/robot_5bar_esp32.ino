// ============================================================================
//  ROBOT PARALELO 5 BARRAS  -  FIRMWARE ESP32-S3   v1.4 (EKF + IMU + prealimentacion)
//  Servos integrados iHSV57 (Step/Dir).  El servo cierra su propio lazo;
//  este firmware genera trayectoria + pasos coordinados y la seguridad.
//
//  v1.3: misma generacion de pulsos que la 1.x (FastAccelStepper, streaming de
//  4 ms).  Se agrega un EKF que corre en la tarea de la IMU (100 Hz, nucleo 0)
//  y estima angulos, velocidades y perturbaciones de los brazos; una ley de
//  correccion (inyeccion de amortiguamiento con filtro de lavado, o LQR) calcula
//  Δq que se SUMA a los pulsos.  Arranca APAGADA: se comporta igual que la 1.x
//  hasta que la app envia "CTL MODO 1" (sombra) o "CTL MODO 2" (activa).
//
//  v1.4: prealimentacion por modelo ("CTL FF 1").  En cada tick el comando es
//  el plan adelantado tau mas K^-1 [M q̈ + C q̇ + B (1 − ff) q̇]: el par que el
//  servo necesitara, convertido en desplazamiento de la referencia.  Con FF 0
//  los pulsos son identicos a los de la 1.3.  La linea I agrega el plan y lo
//  que se sumo a los pulsos (prealimentacion y correccion).
//
//  DEPENDENCIAS (Library Manager de Arduino):
//    - FastAccelStepper   (gin66)
//  PLACA:  ESP32-S3 (Arduino-ESP32 core).  Serial = USB CDC nativo.
//
//  Coloca en la MISMA carpeta que este .ino: kinematics_5bar.h, imu_bno055.h,
//  control_imu.h, ekf_5b.h y lqr_tabla.h.
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
//      PATH <n> x1 y1 x2 y2 ...  recorrer n puntos (max 130). Cerrar = repetir 1er punto
//                                Movimiento CONTINUO: solo frena en esquinas
//                                (> CORNER_DEG); los puntos intermedios no paran.
//      PBEGIN n / PADD x y ... / PRUN   igual que PATH pero cargando la figura
//                                en lineas cortas con confirmacion (lo usa la GUI)
//      MODE                      -> "MODE n" (modo de ensamblaje activo, 1..4)
//      MODE <n>                  cambia el modo de ensamblaje (codos).  Si hay cero
//                                fijado, MUEVE los brazos (movimiento articular
//                                verificado) manteniendo el efector en su punto.
//      STOP                      paro inmediato (aborta movimiento)
//      STATUS                    pide un STATE inmediato
//      IMU                       -> "IMU ok enviando addr=.. cal=.. lecturas=.. ..." (estado de la BNO055)
//      IMU 1 / IMU 0             activa / detiene el envio de muestras de la IMU (lineas I)
//      CTL                       -> "CTL modo=.. ley=.. alfa=.. zona=.. dmax=.. lavado=.. ekf=.. t_ekf=ult/max"
//                                (t_ekf: microsegundos que tarda el EKF + ley en la tarea de la IMU)
//      CTL MODO 0|1|2            correccion con IMU: 0 apagada, 1 sombra (calcula y registra,
//                                no aplica), 2 activa (suma Δq a los pulsos)
//      CTL LEY ALFA|LQR / CTL ALFA v / CTL ZONA gps / CTL DMAX g / CTL LAVADO s
//      CTL GRAV cx cy p1 p2 / CTL RESET       (ver control_imu.h)
//    ESP32 -> Host:
//      OK <msg> / ERR <msg>
//      EVT HOMED | DONE | LIMIT
//      EVT CTL_OFF <motivo>      la correccion se paso sola a SOMBRA (IMU invalida, sin lectura,
//                                innovacion grande o falla del EKF)
//      STATE <estado> <x> <y> <th1deg> <th2deg> <moving>   (telemetria ~20 Hz)
//      MODE <n>                  modo activo (tras MODE, SETHOME o cambio de modo)
//      I t_us q1 q2 ax ay az gx gy gz rumbo roll pitch cal   muestra de la IMU a 100 Hz (con IMU 1)
//                                q1, q2: angulos de los motores en milesimas de grado, en el mismo
//                                instante de la lectura. acel 100 LSB = 1 m/s2, gir 16 LSB = 1 grado/s,
//                                Euler 16 LSB = 1 grado.  Mientras se envia la IMU, STATE baja a 20 Hz.
//                                v1.3 agrega 10 campos:  q1h q2h (EKF, milesimas de grado)
//                                w1h w2h (centesimas de grado/s)  d1 d2 (decimas de mN·m)
//                                dl1 dl2 (correccion calculada, milesimas de grado)
//                                inov (innovacion del giroscopio, centesimas de grado/s)
//                                estado (bit0-1 modo, bit2 EKF valido, bit3 aplicando)
//                                v1.4 agrega 6 campos:  q1p q2p (plan en el instante de la lectura,
//                                milesimas de grado)  ff1 ff2 (prealimentacion sumada)  da1 da2
//                                (correccion de la IMU realmente sumada), milesimas de grado
// ============================================================================
#include <Arduino.h>
#include <FastAccelStepper.h>
#include "kinematics_5bar.h"
#include "imu_bno055.h"            // IMU BNO055 (etapa 1): SDA GPIO 8, SCL GPIO 9
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
#define PIN_PUL2   15     // STEP eje 2  -> PUL- servo 2
#define PIN_DIR2   16     // DIR  eje 2  -> DIR- servo 2

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
//  El utillaje fija tambien el MODO DE ENSAMBLAJE fisico: SETHOME lo detecta
//  y lo activa (con 90/90 y la geometria actual -> modo 1, codos afuera).
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
//  Calculados para L1=L4=0.12, L2=L3=0.18, D=0.15 cubriendo la zona y>0 en los
//  modos 1, 3 y 4 (los angulos son continuos, sin envolver a 0..360).
//  Con esta geometria los angulos van de -168 a 345 grados en esos modos.
//  AJUSTALOS a los topes mecanicos reales de tu montaje (choques con la base,
//  cables, etc.): son la ultima proteccion si algo se calcula mal.
// ============================================================================
const float TH1_MIN_DEG = -170.0f, TH1_MAX_DEG = 350.0f;
const float TH2_MIN_DEG = -170.0f, TH2_MAX_DEG = 350.0f;

// ============================================================================
//  MODO DE ENSAMBLAJE (ver kinematics_5bar.h)
//    1 = codos afuera   2 = codos adentro   3 = izq afuera/der adentro
//    4 = izq adentro/der afuera
//  Al cambiar de modo con el cero fijado, el robot hace un movimiento
//  ARTICULAR (no cartesiano) desde los angulos actuales a los del nuevo modo
//  para el mismo punto.  Antes de moverse se simula el recorrido y se rechaza
//  si: la cadena se abre, cruza una singularidad paralela (|sin| < SWITCH_SIN_MIN),
//  el efector baja de SWITCH_Y_MIN (choque con la base), viola limites o no
//  vuelve al mismo punto.  Si el cambio directo no es seguro, se prueba pasando
//  por un modo intermedio (ej. 1 -> 3 -> 2).
// ============================================================================
const int   DEFAULT_MODE    = 1;
const float JOINT_VMAX      = 1.0f;   // rad/s   velocidad articular del cambio de modo
const float JOINT_AMAX      = 2.0f;   // rad/s2
const float SWITCH_SIN_MIN  = 0.05f;
const float SWITCH_Y_MIN    = 0.02f;  // m
const int   SWITCH_SAMPLES  = 200;

// ============================================================================
//  PARAMETROS DE MOVIMIENTO Y EJECUCION
// ============================================================================
float    g_Vmax = 0.10f;     // m/s   (modificable por SETV; arranca conservador)
float    g_Amax = 0.20f;     // m/s2
const uint32_t TICK_US   = 4000;     // 250 Hz: actualizacion del setpoint
const float    MAX_HZ    = 60000.0f; // tope de FastAccelStepper (no debe limitar)
const float    MAX_ACC   = 400000.0f;
const uint32_t TELEM_MS  = 20;       // 50 Hz de telemetria (antes 50 ms = 20 Hz)

// Topes de seguridad para SETV (la app permite hasta 0.5 m/s y 3 m/s2).
// Suban g_Vmax/g_Amax por pasos y vigilen la alarma del driver.
const float VMAX_LIMIT = 0.50f;      // m/s
const float AMAX_LIMIT = 3.00f;      // m/s2

// ============================================================================
//  TRAYECTORIA CONTINUA
//  Una figura se divide en TRAZOS que se recorren SIN detenerse.  El robot solo
//  frena (v = 0) en las ESQUINAS: waypoints donde la direccion cambia mas de
//  CORNER_DEG.  Un circulo es UN trazo a velocidad constante; un cuadrado son
//  4 trazos (para en sus 4 vertices, no en cada punto intermedio).
//  Debe coincidir con CORNER_DEG de la GUI (solo para la vista previa).
// ============================================================================
const float CORNER_DEG   = 30.0f;
const float MIN_WP_DIST  = 0.0002f;   // 0.2 mm: puntos mas cercanos se fusionan
const float APPROACH_MIN = 0.0005f;   // si el efector esta a >0.5 mm del 1er punto, primero se acerca

// Seguimiento por streaming: cada tick se comanda el punto donde debe estar el
// efector LOOKAHEAD_TICKS mas adelante, con la velocidad de pasos justa para
// llegar en ese tiempo.  El motor nunca alcanza el objetivo entre ticks, asi
// que no frena/arranca cada 4 ms: tren de pulsos continuo.
const int   LOOKAHEAD_TICKS = 3;      // horizonte H = 3 x 4 ms = 12 ms
const float MIN_STEP_HZ     = 200.0f;

#define MAX_WP_HOST 130               // puntos que acepta el comando PATH
#define MAX_WP (MAX_WP_HOST + 1)      // +1 punto de aproximacion (posicion actual)
float    g_wx[MAX_WP], g_wy[MAX_WP];
float    g_cum[MAX_WP];               // longitud de arco acumulada hasta cada waypoint
int      g_nWp = 0;
int      g_forceStop = -1;            // waypoint donde se fuerza parada (fin de la aproximacion)
int      g_upN = 0, g_upCount = 0;    // carga por partes (PBEGIN/PADD/PRUN)

// ============================================================================
//  ESTADO
// ============================================================================
enum State { ST_INIT, ST_IDLE, ST_JOGGING, ST_MOVING, ST_HALTED };
State g_state = ST_INIT;
bool  g_homed = false;

float g_curX = 0, g_curY = 0;   // posicion cartesiana comandada actual
float g_q1 = 0, g_q2 = 0;       // ultimos angulos del PLAN (rad, continuos; sin la correccion de la IMU)
float g_dA1 = 0, g_dA2 = 0;     // correccion de la IMU sumada en el ultimo tick (rad)
float g_ff1 = 0, g_ff2 = 0;     // 1.4: desplazamiento de la prealimentacion sumado en el ultimo tick (rad)
int   g_mode = DEFAULT_MODE;    // modo de ensamblaje activo

// cambio de modo: hasta 2 movimientos articulares encadenados
struct JointSeg { float q1a, q2a, q1b, q2b, T; int modeAfter; };
JointSeg g_js[2];
int      g_nJs = 0, g_jsIdx = 0;
uint32_t g_jsStartUs = 0;
bool     g_jointMove = false;

// trazo en ejecucion (waypoints g_strokeA .. g_strokeB)
int           g_strokeA = 0, g_strokeB = 0;
int           g_segIdx  = 0;          // segmento actual dentro del trazo (busqueda incremental)
SmoothProfile g_prof;
uint32_t      g_strokeStartUs = 0;
bool          g_finishing = false;    // ultimo punto comandado, esperando que los motores lleguen
long          g_tgt1 = 0, g_tgt2 = 0; // objetivo final en pasos (fase de llegada)
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

float motorAngle1() { return (st1 ? st1->getCurrentPosition() : 0) / (DIR_SIGN1 * STEPS_PER_RAD); }
float motorAngle2() { return (st2 ? st2->getCurrentPosition() : 0) / (DIR_SIGN2 * STEPS_PER_RAD); }

// La tarea de la IMU (nucleo 0) pide los angulos de los motores en el instante de cada lectura
void angulosParaImu(int32_t &q1_mdeg, int32_t &q2_mdeg) {
    q1_mdeg = lroundf(degrees(motorAngle1()) * 1000.0f);
    q2_mdeg = lroundf(degrees(motorAngle2()) * 1000.0f);
}

void sendState() {
    float t1 = motorAngle1();
    float t2 = motorAngle2();
    // posicion REAL del efector (FK de los pasos emitidos), no la comandada
    float x = g_curX, y = g_curY;
    if (g_homed) {
        float fx, fy;   // rama de la FK por continuidad con la ultima posicion
        if (fk5barNear(t1, t2, g_curX, g_curY, fx, fy)) { x = fx; y = fy; }
    }
    bool moving = (g_state == ST_MOVING) || (g_state == ST_JOGGING);
    const char* s = (g_state==ST_IDLE)?"IDLE":(g_state==ST_JOGGING)?"JOGGING":
                    (g_state==ST_MOVING)?"MOVING":(g_state==ST_HALTED)?"HALTED":"INIT";
    Serial.printf("STATE %s %.4f %.4f %.2f %.2f %d\n",
                  s, x, y, degrees(t1), degrees(t2), moving ? 1 : 0);
}

// Tras un paro brusco, la posicion comandada puede ir por delante de la real.
// La correccion de la IMU que llevaba sumada queda absorbida y el EKF se reinicia.
void syncPoseFromMotors() {
    ctl::pedirReinicio();
    g_dA1 = g_dA2 = 0;
    if (!g_homed) return;
    g_q1 = motorAngle1(); g_q2 = motorAngle2();
    float fx, fy;
    if (fk5barNear(g_q1, g_q2, g_curX, g_curY, fx, fy)) { g_curX = fx; g_curY = fy; }
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
    g_finishing = false;
    g_jointMove = false;
    syncPoseFromMotors();
    Serial.printf("EVT %s\n", reason);
}

// IK + chequeos de seguridad -> objetivo en pasos. false si no es seguro.
bool cartesianToAngles(float x, float y, float &t1, float &t2) {
    if (!ik5bar(x, y, g_mode, t1, t2)) return false;
    if (singularityMeasure(x, y, t1, t2) < SIN_MIN) return false;
    t1 = wrapNear(t1, g_q1);          // continuidad: nunca saltar 360 deg
    t2 = wrapNear(t2, g_q2);
    return jointLimitsOK(t1, t2);
}

// Manda un eje hacia 'target' con la velocidad de pasos necesaria para llegar
// en 'horizon' segundos.  Es un seguimiento proporcional con prealimentacion:
// a velocidad constante el error en regimen es cero y la tasa de pulsos es
// estable (no hay rafagas + esperas).
void streamAxis(FastAccelStepper* st, long target, float horizon) {
    long dist = labs(target - st->getCurrentPosition());
    float hz = (float)dist / horizon;
    if (hz < MIN_STEP_HZ) hz = MIN_STEP_HZ;
    if (hz > MAX_HZ)      hz = MAX_HZ;
    st->setSpeedInHz((uint32_t)hz);
    st->moveTo(target);
}

// Comanda el punto del PLAN (t1, t2) que debe alcanzarse en 'horizon' segundos:
//  - lo registra para la tarea de la IMU (la ley compara el estado estimado con el plan),
//  - le suma la correccion Δq vigente (0 si esta apagada o en sombra).
//  - 1.4: le suma tambien el desplazamiento de la prealimentacion (f1, f2), 0 si esta apagada.
void streamPlan(float t1, float t2, float x, float y, float horizon, float f1, float f2) {
    ctl::registrarPlan(micros() + (uint32_t)(horizon * 1e6f), t1, t2, x, y);
    float d1, d2;
    ctl::dqActual(d1, d2);
    g_dA1 = d1; g_dA2 = d2;
    g_ff1 = f1; g_ff2 = f2;
    ctl::anotarOffsets(f1, f2, d1, d2);
    streamAxis(st1, stepsAxis1(t1 + f1 + d1), horizon);
    streamAxis(st2, stepsAxis2(t2 + f2 + d2), horizon);
}

// Mueve ambos ejes hacia (x,y) si es seguro. false si no.
// cmd (1.4): angulos a comandar con la prealimentacion; nullptr = el plan tal cual.
bool commandCartesian(float x, float y, float horizon, const float *cmd) {
    float t1, t2;
    if (!cartesianToAngles(x, y, t1, t2)) return false;
    if (cmd && jointLimitsOK(cmd[0], cmd[1])) streamPlan(t1, t2, x, y, horizon, cmd[0] - t1, cmd[1] - t2);
    else streamPlan(t1, t2, x, y, horizon, 0.0f, 0.0f);
    g_tgt1 = stepsAxis1(t1); g_tgt2 = stepsAxis2(t2);     // objetivo del plan (sin correccion)
    g_q1 = t1; g_q2 = t2;
    g_curX = x; g_curY = y;
    return true;
}

// Mueve ambos ejes a unos angulos (movimiento articular). Actualiza (x,y) por FK.
void commandJoints(float t1, float t2, float horizon) {
    float fx, fy;
    if (fk5barNear(t1, t2, g_curX, g_curY, fx, fy)) { g_curX = fx; g_curY = fy; }
    streamPlan(t1, t2, g_curX, g_curY, horizon, 0.0f, 0.0f);
    g_tgt1 = stepsAxis1(t1); g_tgt2 = stepsAxis2(t2);
    g_q1 = t1; g_q2 = t2;
}

// true si los angulos actuales son la solucion del modo activo en el punto actual
bool poseMatchesMode() {
    return anglesMatchMode(g_curX, g_curY, g_q1, g_q2, g_mode, radians(2.0f));
}

bool validatePath() {
    for (int i = 0; i < g_nWp; i++) {
        if (!isReachableAndSafe(g_wx[i], g_wy[i], g_mode)) {
            Serial.printf("ERR punto %d (%.3f,%.3f) inalcanzable o singular\n",
                          i, g_wx[i], g_wy[i]);
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
//  PREPARACION DE LA RUTA
//   1) fusiona puntos repetidos (p.ej. el punto de cierre si coincide)
//   2) si el efector no esta en el 1er punto, antepone su posicion actual
//      (antes el robot "saltaba" al 1er punto a velocidad maxima)
//   3) calcula la longitud de arco acumulada
// ---------------------------------------------------------------------------
void preparePath() {
    int n = 0;
    for (int i = 0; i < g_nWp; i++) {
        if (n > 0 && hypotf(g_wx[i] - g_wx[n-1], g_wy[i] - g_wy[n-1]) < MIN_WP_DIST) continue;
        g_wx[n] = g_wx[i]; g_wy[n] = g_wy[i]; n++;
    }
    g_nWp = n;

    g_forceStop = -1;
    if (g_nWp > 0 && g_nWp < MAX_WP &&
        hypotf(g_wx[0] - g_curX, g_wy[0] - g_curY) > APPROACH_MIN) {
        for (int i = g_nWp; i > 0; i--) { g_wx[i] = g_wx[i-1]; g_wy[i] = g_wy[i-1]; }
        g_wx[0] = g_curX; g_wy[0] = g_curY;
        g_nWp++;
        g_forceStop = 1;          // llega al inicio de la figura, se detiene y arranca
    }

    g_cum[0] = 0.0f;
    for (int i = 1; i < g_nWp; i++)
        g_cum[i] = g_cum[i-1] + hypotf(g_wx[i] - g_wx[i-1], g_wy[i] - g_wy[i-1]);
}

// true si el robot debe DETENERSE en el waypoint i (extremos o esquina)
bool isStopPoint(int i) {
    if (i <= 0 || i >= g_nWp - 1) return true;
    if (i == g_forceStop) return true;
    float ax = g_wx[i]   - g_wx[i-1], ay = g_wy[i]   - g_wy[i-1];
    float bx = g_wx[i+1] - g_wx[i],   by = g_wy[i+1] - g_wy[i];
    float la = hypotf(ax, ay), lb = hypotf(bx, by);
    if (la < 1e-9f || lb < 1e-9f) return false;
    float c = (ax * bx + ay * by) / (la * lb);      // coseno del giro
    return c < cosf(CORNER_DEG * PI_F / 180.0f);
}

int countStrokes() {
    int k = 0;
    for (int i = 1; i < g_nWp; i++) if (isStopPoint(i)) k++;
    return k;
}

// Arranca el trazo que empieza en el waypoint a (termina en la siguiente parada)
void startStroke(int a, uint32_t startUs) {
    int b = a + 1;
    while (!isStopPoint(b)) b++;
    g_strokeA = a;
    g_strokeB = b;
    g_segIdx  = a;
    g_prof = makeProfile(g_cum[b] - g_cum[a], g_Vmax, g_Amax);
    g_strokeStartUs = startUs;
}

// Punto de la polilinea a una distancia de arco s (dentro del trazo actual)
void pointAtArc(float s, float &x, float &y) {
    while (g_segIdx < g_strokeB - 1 && g_cum[g_segIdx + 1] < s) g_segIdx++;
    int i = g_segIdx;
    float len = g_cum[i+1] - g_cum[i];
    float u = (len > 1e-9f) ? (s - g_cum[i]) / len : 1.0f;
    u = clampf(u, 0.0f, 1.0f);
    x = g_wx[i] + (g_wx[i+1] - g_wx[i]) * u;
    y = g_wy[i] + (g_wy[i+1] - g_wy[i]) * u;
}

// ---------------------------------------------------------------------------
//  PREALIMENTACION (v1.4)
//  El servo sigue al comando con un tiempo muerto tau y necesita "error" para
//  generar par.  Con el modelo identificado se manda por adelantado:
//      q_c(t) = q_d(t + tau) + K^-1 [M q̈_d + C q̇_d + B (1 − ff) q̇_d]
//  q_d, q̇_d y q̈_d se sacan del plan en t + tau (diferencias centradas de ±FF_DT).
// ---------------------------------------------------------------------------
const float FF_DT = 0.012f;           // s: paso de las diferencias (suaviza los vertices de la polilinea)

// Punto del plan en el instante tl (s, contado desde el inicio del trazo actual),
// sin tocar el estado del planificador.  Sigue a los trazos siguientes si hace falta.
void planPointAhead(float tl, float &x, float &y) {
    int a = g_strokeA, b = g_strokeB;
    SmoothProfile p = g_prof;
    while (tl >= p.T && b < g_nWp - 1) {
        tl -= p.T;
        a = b;
        b = a + 1;
        while (!isStopPoint(b)) b++;
        p = makeProfile(g_cum[b] - g_cum[a], g_Vmax, g_Amax);
    }
    if (tl > p.T) tl = p.T;
    if (tl < 0.0f) tl = 0.0f;
    float s = g_cum[a] + profilePos(p, tl);
    int i = (a == g_strokeA) ? g_segIdx : a;
    if (i > b - 1) i = b - 1;
    while (i > a && g_cum[i] > s) i--;
    while (i < b - 1 && g_cum[i + 1] < s) i++;
    float len = g_cum[i + 1] - g_cum[i];
    float u = (len > 1e-9f) ? (s - g_cum[i]) / len : 1.0f;
    u = clampf(u, 0.0f, 1.0f);
    x = g_wx[i] + (g_wx[i + 1] - g_wx[i]) * u;
    y = g_wy[i] + (g_wy[i + 1] - g_wy[i]) * u;
}

bool anglesAt(float x, float y, float &t1, float &t2) {
    if (!ik5bar(x, y, g_mode, t1, t2)) return false;
    t1 = wrapNear(t1, g_q1);
    t2 = wrapNear(t2, g_q2);
    return true;
}

// tc: instante del plan que se va a comandar (desde el inicio del trazo).  cmd: angulos a mandar.
bool feedforward(float tc, float cmd[2]) {
    ekf5b::Param p = ctl::paramActual();
    float ta = tc + p.tau;
    float xa, ya, xb, yb, xc, yc, a1, a2, b1, b2, c1, c2;
    planPointAhead(ta - FF_DT, xa, ya);
    planPointAhead(ta, xb, yb);
    planPointAhead(ta + FF_DT, xc, yc);
    if (!anglesAt(xa, ya, a1, a2) || !anglesAt(xb, yb, b1, b2) || !anglesAt(xc, yc, c1, c2)) return false;
    float q[2]   = {b1, b2};
    float qd[2]  = {(c1 - a1) / (2 * FF_DT), (c2 - a2) / (2 * FF_DT)};
    float qdd[2] = {(c1 - 2 * b1 + a1) / (FF_DT * FF_DT), (c2 - 2 * b2 + a2) / (FF_DT * FF_DT)};
    float u[2];
    if (!ekf5b::prealimentacion(p, q, qd, qdd, u)) return false;
    float g = ctl::g_ffGan, m = ctl::g_ffMax;
    for (int i = 0; i < 2; i++) u[i] = clampf(g * u[i], -m, m);
    cmd[0] = b1 + u[0];
    cmd[1] = b2 + u[1];
    return true;
}

void startPath() {
    if (!g_homed)   { Serial.println("ERR no homed"); return; }
    if (g_nWp < 2)  { Serial.println("ERR path needs >= 2 points"); return; }
    if (g_jointMove) { Serial.println("ERR busy (cambiando de modo)"); return; }
    if (g_state != ST_MOVING && !poseMatchesMode()) {
        // los brazos no estan en el modo activo: el 1er paso seria un salto
        Serial.printf("ERR los brazos no estan en el modo %d (detectado: %d). Usa MODE\n",
                      g_mode, detectMode(g_curX, g_curY, g_q1, g_q2));
        return;
    }
    if (!validatePath()) return;
    preparePath();
    if (g_nWp < 2) {                       // ya estaba en el destino
        g_state = ST_IDLE;
        Serial.println("OK already there");
        Serial.println("EVT DONE");
        return;
    }
    g_finishing = false;
    uint32_t now = micros();
    startStroke(0, now);
    g_state = ST_MOVING;
    g_lastTickUs = now;
    Serial.printf("OK moving %d pts, %d trazos, %.3f m\n",
                  g_nWp, countStrokes(), g_cum[g_nWp - 1]);
}

void updateMoving() {
    uint32_t now = micros();
    if ((uint32_t)(now - g_lastTickUs) < TICK_US) return;
    g_lastTickUs = now;

    const float H = LOOKAHEAD_TICKS * TICK_US * 1e-6f;   // horizonte (s)

    // --- fase final: el ultimo punto ya esta comandado, esperar llegada ---
    if (g_finishing) {
        streamPlan(g_q1, g_q2, g_curX, g_curY, H, 0.0f, 0.0f);
        long p1 = st1->getCurrentPosition(), p2 = st2->getCurrentPosition();
        bool llego;
        if (g_dA1 == 0.0f && g_dA2 == 0.0f) {
            // sin correccion: igual que la 1.x (motores detenidos exactamente en el objetivo)
            llego = !st1->isRunning() && !st2->isRunning() && p1 == g_tgt1 && p2 == g_tgt2;
        } else {
            // con correccion el objetivo se mueve cada 10 ms: basta con estar dentro de la banda
            // de la correccion alrededor del objetivo del plan (asi EVT DONE llega al mismo
            // tiempo que sin IMU y la correccion sigue actuando en reposo, ver updateIdle)
            long b1 = labs(stepsAxis1(g_q1 + g_dA1) - g_tgt1) + 2;
            long b2 = labs(stepsAxis2(g_q2 + g_dA2) - g_tgt2) + 2;
            llego = labs(p1 - g_tgt1) <= b1 && labs(p2 - g_tgt2) <= b2;
        }
        if (llego) {
            g_finishing = false;
            g_state = ST_IDLE;
            if (g_jointMove) {
                g_jointMove = false;
                Serial.printf("OK modo %d activo\n", g_mode);
                Serial.printf("MODE %d\n", g_mode);
            }
            Serial.println("EVT DONE");
        }
        return;
    }

    // --- cambio de modo: movimiento articular con perfil quintico ---
    if (g_jointMove) {
        float tc = (int32_t)(now - g_jsStartUs) * 1e-6f + H;
        while (tc >= g_js[g_jsIdx].T && g_jsIdx + 1 < g_nJs) {
            g_mode = g_js[g_jsIdx].modeAfter;            // modo intermedio alcanzado
            tc -= g_js[g_jsIdx].T;
            g_jsStartUs += (uint32_t)(g_js[g_jsIdx].T * 1e6f);
            g_jsIdx++;
        }
        const JointSeg &j = g_js[g_jsIdx];
        float u = quinticS(j.T > 0 ? tc / j.T : 1.0f);
        commandJoints(j.q1a + (j.q1b - j.q1a) * u, j.q2a + (j.q2b - j.q2a) * u, H);
        if (tc >= j.T && g_jsIdx + 1 >= g_nJs) {
            g_mode = j.modeAfter;
            g_finishing = true;
        }
        return;
    }

    // instante (dentro del trazo) al que corresponde el punto que comandamos
    float tc = (int32_t)(now - g_strokeStartUs) * 1e-6f + H;

    // pasar al siguiente trazo sin perder tiempo (linea de tiempo continua)
    while (tc >= g_prof.T && g_strokeB < g_nWp - 1) {
        tc -= g_prof.T;
        startStroke(g_strokeB, g_strokeStartUs + (uint32_t)(g_prof.T * 1e6f));
    }

    float s = g_cum[g_strokeA] + profilePos(g_prof, tc);
    float x, y;
    pointAtArc(s, x, y);
    float cmd[2];
    bool ff = ctl::g_ffOn && feedforward(tc, cmd);
    if (!commandCartesian(x, y, H, ff ? cmd : nullptr)) { haltAll("LIMIT"); return; }

    if (tc >= g_prof.T && g_strokeB >= g_nWp - 1) g_finishing = true;
}

// ---------------------------------------------------------------------------
//  REPOSO (v1.3): con la correccion encendida se sigue registrando el plan y
//  sumando Δq cada 4 ms, para amortiguar la vibracion despues de EVT DONE.
//  Con la correccion apagada no hace nada (igual que la 1.x).
// ---------------------------------------------------------------------------
void updateIdle() {
    if (!g_homed || ctl::g_modo == ctl::APAGADO) return;
    uint32_t now = micros();
    if ((uint32_t)(now - g_lastTickUs) < TICK_US) return;
    g_lastTickUs = now;
    streamPlan(g_q1, g_q2, g_curX, g_curY, LOOKAHEAD_TICKS * TICK_US * 1e-6f, 0.0f, 0.0f);
}

// ---------------------------------------------------------------------------
//  CERO POR UTILLAJE: fija la posicion actual a (th1deg, th2deg).
//  Con el utillaje puesto y los servos habilitados, redefine el contador de
//  pasos: a partir de aqui el robot sabe su angulo absoluto.  No mueve nada.
// ---------------------------------------------------------------------------
void setHome(float th1deg, float th2deg) {
    float t1 = radians(th1deg), t2 = radians(th2deg);
    float xs[2], ys[2];
    if (!fk5barBoth(t1, t2, xs, ys)) {
        Serial.println("ERR sethome: con esos angulos los brazos no pueden cerrar la cadena");
        return;
    }
    st1->setCurrentPosition(stepsAxis1(t1));
    st2->setCurrentPosition(stepsAxis2(t2));
    g_q1 = t1; g_q2 = t2;
    g_dA1 = g_dA2 = 0;
    ctl::pedirReinicio();          // los contadores de pasos saltaron: el EKF empieza de nuevo

    // Rama de la FK: la que corresponde al modo activo; si ninguna, la de
    // efector mas ALTO (la fisica con el utillaje sobre la base) y se adopta
    // el modo que corresponde a esa pose.
    int k = -1;
    for (int i = 0; i < 2; i++)
        if (anglesMatchMode(xs[i], ys[i], t1, t2, g_mode, 1e-3f)) { k = i; break; }
    if (k < 0) {
        k = (ys[0] >= ys[1]) ? 0 : 1;
        int m = detectMode(xs[k], ys[k], t1, t2);
        if (m > 0) {
            Serial.printf("OK el utillaje deja el robot en el modo %d (antes %d): modo %d activo\n",
                          m, g_mode, m);
            g_mode = m;
        } else {
            Serial.println("WARN el cero esta en una singularidad de cadena; ningun modo coincide");
        }
    }
    g_curX = xs[k]; g_curY = ys[k];
    g_homed = true;
    if (g_state == ST_HALTED || g_state == ST_INIT) g_state = ST_IDLE;
    Serial.printf("OK home set th1=%.2f th2=%.2f -> x=%.4f y=%.4f (modo %d)\n",
                  th1deg, th2deg, g_curX, g_curY, g_mode);
    Serial.printf("MODE %d\n", g_mode);
    Serial.println("EVT HOMED");
}

// ---------------------------------------------------------------------------
//  CAMBIO DE MODO DE ENSAMBLAJE
// ---------------------------------------------------------------------------
// Simula el movimiento articular lineal qa -> qb partiendo del efector en
// (x0,y0) y siguiendo la rama fisica de la FK.  true si es seguro y termina
// otra vez en (x0,y0).
bool jointSegSafe(float q1a, float q2a, float q1b, float q2b, float x0, float y0) {
    float px = x0, py = y0;
    for (int k = 1; k <= SWITCH_SAMPLES; k++) {
        float u = (float)k / SWITCH_SAMPLES;
        float t1 = q1a + (q1b - q1a) * u, t2 = q2a + (q2b - q2a) * u;
        float nx, ny;
        if (!fk5barNear(t1, t2, px, py, nx, ny)) return false;    // cadena abierta
        px = nx; py = ny;
        if (py < SWITCH_Y_MIN) return false;                        // choque con la base
        if (singularityMeasure(px, py, t1, t2) < SWITCH_SIN_MIN) return false;
        if (!jointLimitsOK(t1, t2)) return false;
    }
    return hypotf(px - x0, py - y0) < 0.001f;
}

float jointSegTime(float q1a, float q2a, float q1b, float q2b) {
    float dq = fmaxf(fabsf(q1b - q1a), fabsf(q2b - q2a));
    return segDuration(dq, JOINT_VMAX, JOINT_AMAX);   // misma formula quintica
}

void requestMode(int newMode) {
    int s1, s2;
    if (!modeSigns(newMode, s1, s2)) { Serial.println("ERR modo 1..4"); Serial.printf("MODE %d\n", g_mode); return; }
    if (g_state == ST_MOVING || g_state == ST_JOGGING) { Serial.println("ERR busy"); Serial.printf("MODE %d\n", g_mode); return; }

    if (!g_homed) {                      // sin cero: solo se elige el modo (no mueve)
        g_mode = newMode;
        Serial.printf("OK modo %d (sin cero: no se mueve; SETHOME confirmara el modo real)\n", g_mode);
        Serial.printf("MODE %d\n", g_mode);
        return;
    }
    syncPoseFromMotors();
    if (newMode == g_mode && poseMatchesMode()) {
        Serial.printf("OK ya en modo %d\n", g_mode);
        Serial.printf("MODE %d\n", g_mode);
        return;
    }
    float x0 = g_curX, y0 = g_curY;
    float qb1, qb2;
    if (!ik5bar(x0, y0, newMode, qb1, qb2) || singularityMeasure(x0, y0, qb1, qb2) < SIN_MIN) {
        Serial.printf("ERR modo %d: el punto actual (%.3f,%.3f) es inalcanzable o singular en ese modo\n",
                      newMode, x0, y0);
        Serial.printf("MODE %d\n", g_mode);
        return;
    }
    float qa1 = g_q1, qa2 = g_q2;

    // 1) cambio directo
    g_nJs = 0;
    float b1 = wrapNear(qb1, qa1), b2 = wrapNear(qb2, qa2);
    if (jointSegSafe(qa1, qa2, b1, b2, x0, y0)) {
        g_js[0] = {qa1, qa2, b1, b2, jointSegTime(qa1, qa2, b1, b2), newMode};
        g_nJs = 1;
    } else {
        // 2) pasando por un modo intermedio
        for (int m = 1; m <= 4 && g_nJs == 0; m++) {
            if (m == newMode || m == g_mode) continue;
            float mq1, mq2;
            if (!ik5bar(x0, y0, m, mq1, mq2) || singularityMeasure(x0, y0, mq1, mq2) < SIN_MIN) continue;
            mq1 = wrapNear(mq1, qa1); mq2 = wrapNear(mq2, qa2);
            float c1 = wrapNear(qb1, mq1), c2 = wrapNear(qb2, mq2);
            if (jointSegSafe(qa1, qa2, mq1, mq2, x0, y0) && jointSegSafe(mq1, mq2, c1, c2, x0, y0)) {
                g_js[0] = {qa1, qa2, mq1, mq2, jointSegTime(qa1, qa2, mq1, mq2), m};
                g_js[1] = {mq1, mq2, c1, c2, jointSegTime(mq1, mq2, c1, c2), newMode};
                g_nJs = 2;
            }
        }
    }
    if (g_nJs == 0) {
        Serial.printf("ERR cambio a modo %d no es seguro desde (%.3f,%.3f): cruzaria una "
                      "singularidad o los brazos bajarian a la base. Mueve el efector a otro punto\n",
                      newMode, x0, y0);
        Serial.printf("MODE %d\n", g_mode);
        return;
    }
    g_jsIdx = 0;
    g_jointMove = true;
    g_finishing = false;
    uint32_t now = micros();
    g_jsStartUs = now;
    g_lastTickUs = now;
    g_state = ST_MOVING;
    float Ttot = g_js[0].T + (g_nJs > 1 ? g_js[1].T : 0.0f);
    if (g_nJs == 1) Serial.printf("OK cambiando a modo %d (%.1f s)\n", newMode, Ttot);
    else            Serial.printf("OK cambiando a modo %d via modo %d (%.1f s)\n", newMode, g_js[0].modeAfter, Ttot);
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

    if (!strcmp(tok, "IMU")) {
        char* a = strtok(NULL, " \t\r\n");
        if (!a) { imu::reportarEstado(); return; }
        if (!imu::g_ok) { Serial.println("ERR imu no disponible"); return; }
        bool on = atoi(a) != 0;
        imu::activarEnvio(on);
        Serial.printf("OK imu %d\n", on ? 1 : 0);
        return;
    }

    if (!strcmp(tok, "CTL")) { ctl::comando(imu::g_ok); return; }

    if (!strcmp(tok, "MODE")) {
        char* a = strtok(NULL, " \t\r\n");
        if (!a) { Serial.printf("MODE %d\n", g_mode); return; }
        requestMode(atoi(a));
        return;
    }

    if (!strcmp(tok, "STOP")) {
        if (st1) st1->forceStop();
        if (st2) st2->forceStop();
        g_nWp = 0;
        g_upN = 0;
        g_finishing = false;
        if (g_jointMove) {
            // cambio de modo interrumpido: el modo es el que corresponda a la pose
            g_jointMove = false;
            syncPoseFromMotors();
            int m = detectMode(g_curX, g_curY, g_q1, g_q2);
            if (m > 0) g_mode = m;
            else Serial.println("WARN cambio de modo interrumpido a medias: usa MODE <n> para terminarlo");
            Serial.printf("MODE %d\n", g_mode);
        }
        syncPoseFromMotors();
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
        g_Vmax = clampf(g_Vmax, 0.01f, VMAX_LIMIT);
        g_Amax = clampf(g_Amax, 0.01f, AMAX_LIMIT);
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
        if (n < 2 || n > MAX_WP_HOST) { Serial.printf("ERR n out of range (2..%d)\n", MAX_WP_HOST); return; }
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

    // ---- CARGA DE FIGURA POR PARTES (la usa la GUI) -------------------------
    //  PBEGIN n            -> "OK pbegin n"   prepara recepcion de n puntos
    //  PADD x1 y1 x2 y2 .. -> "OK padd k"     agrega puntos (k = recibidos hasta ahora)
    //  PRUN                -> ejecuta (igual que PATH)
    //  Cada linea es corta (< 200 bytes) y la GUI espera el OK antes de enviar
    //  la siguiente, asi nunca se desborda el buffer serie.
    if (!strcmp(tok, "PBEGIN")) {
        if (g_state == ST_MOVING) { Serial.println("ERR busy"); return; }
        char* a = strtok(NULL, " \t\r\n");
        int n = a ? atoi(a) : 0;
        if (n < 2 || n > MAX_WP_HOST) {
            g_upN = 0;
            Serial.printf("ERR n out of range (2..%d)\n", MAX_WP_HOST); return;
        }
        g_upN = n; g_upCount = 0;
        Serial.printf("OK pbegin %d\n", n);
        return;
    }

    if (!strcmp(tok, "PADD")) {
        if (g_upN == 0) { Serial.println("ERR padd sin pbegin"); return; }
        if (g_state == ST_MOVING) { Serial.println("ERR busy"); return; }
        while (true) {
            char* xs = strtok(NULL, " \t\r\n");
            if (!xs) break;
            char* ys = strtok(NULL, " \t\r\n");
            if (!ys) { g_upN = 0; Serial.println("ERR padd coordenada impar"); return; }
            if (g_upCount >= g_upN) { g_upN = 0; Serial.println("ERR padd demasiados puntos"); return; }
            g_wx[g_upCount] = atof(xs); g_wy[g_upCount] = atof(ys);
            g_upCount++;
        }
        Serial.printf("OK padd %d\n", g_upCount);
        return;
    }

    if (!strcmp(tok, "PRUN")) {
        if (g_upN == 0 || g_upCount != g_upN) {
            Serial.printf("ERR prun: recibidos %d de %d puntos\n", g_upCount, g_upN);
            g_upN = 0; return;
        }
        g_nWp = g_upN;
        g_upN = 0;
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
    // El buffer de recepcion por defecto es de 256 bytes: una figura enviada en
    // una sola linea (PATH, ~600 bytes) se desbordaba y se perdian puntos.
    // Debe llamarse ANTES de Serial.begin().
    Serial.setRxBufferSize(4096);
    // Con la IMU se envian ~7 kB/s: un buffer de salida grande evita que Serial.write
    // bloquee el lazo de trayectoria esperando a que se vacie el puerto.
    Serial.setTxBufferSize(4096);
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

    imu::iniciar(angulosParaImu);      // si no hay BNO055, el robot funciona igual

    g_state = ST_INIT;
    Serial.println("OK ready - 5bar ESP32-S3 v1.4 (geom 120/180/150 mm)");
    Serial.printf("OK PPR=%.0f Vmax=%.3f Amax=%.3f\n", STEPS_PER_REV, g_Vmax, g_Amax);
    Serial.printf("GEOM %.4f %.4f %.4f %.4f %.4f\n", L1, L2, L3, L4, D);
    Serial.printf("MODE %d\n", g_mode);
    ctl::reportar();
}

void loop() {
    pollSerial();

    // la correccion solo actua en movimiento cartesiano y en reposo con cero fijado
    ctl::ponerPermitido(g_homed && !g_jointMove && (g_state == ST_MOVING || g_state == ST_IDLE));

    switch (g_state) {
        case ST_MOVING: updateMoving(); break;
        case ST_IDLE:   updateIdle(); break;
        case ST_JOGGING:
            if (!st1->isRunning() && !st2->isRunning()) {
                st1->setSpeedInHz((uint32_t)MAX_HZ);
                st2->setSpeedInHz((uint32_t)MAX_HZ);
                syncPoseFromMotors();   // el jog movio los brazos: actualizar (x,y)
                g_state = g_homed ? ST_IDLE : ST_INIT;
            }
            break;
        default: break;
    }

    int ev = ctl::tomarEvento();
    if (ev != ctl::EV_NINGUNO) {
        Serial.printf("EVT CTL_OFF %s\n", ctl::nombreEvento(ev));
        ctl::reportar();
    }

    imu::enviarPendientes();

    uint32_t ms = millis();
    uint32_t periodoTelem = imu::enviando() ? 50 : TELEM_MS;   // con IMU, STATE a 20 Hz
    if ((uint32_t)(ms - g_lastTelemMs) >= periodoTelem) {
        g_lastTelemMs = ms;
        sendState();
    }
}

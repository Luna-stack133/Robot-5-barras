// ============================================================================
//  control_imu.h  -  Puente entre la tarea de la IMU (nucleo 0, EKF + ley) y el
//                    lazo de trayectoria (loop, nucleo 1).   Firmware 1.3.
//
//  - loop() registra el PLAN (angulos sin correccion) cada 4 ms en un buffer.
//  - La tarea de la IMU, despues de cada lectura (100 Hz), corre el EKF y la ley,
//    y deja la correccion Δq que loop() suma a los pulsos.
//  - Modos:  0 APAGADO (como el firmware 1.x: no corre el EKF)
//            1 SOMBRA  (corre el EKF y calcula Δq, pero NO lo aplica: se registra)
//            2 ACTIVO  (aplica Δq)
//  - Seguridad: recorte |Δq| ≤ dmax, limite de cambio por muestra, filtro de
//    lavado, rampa a 0 al apagar, y paso automatico a SOMBRA si la IMU deja de
//    leer o entrega lecturas invalidas, o el EKF deja de coincidir con el
//    giroscopio (innovacion grande) o falla.  Si la tarea de la IMU se detiene,
//    loop() deja de sumar la correccion a los 60 ms.
// ============================================================================
#ifndef CONTROL_IMU_H
#define CONTROL_IMU_H

#include <Arduino.h>
#include "ekf_5b.h"

namespace ctl {

enum { APAGADO = 0, SOMBRA = 1, ACTIVO = 2 };
enum { EV_NINGUNO = 0, EV_IMU_INVALIDA = 1, EV_INNOVACION = 2, EV_EKF = 3, EV_SIN_LECTURA = 4 };

const float    INOV_MAX   = 30.0f * 0.017453293f;   // rad/s: innovacion del giroscopio considerada anomala
const int      N_SEGUIDAS = 5;                      // muestras seguidas (50 ms) para desactivar
const uint32_t VIGENCIA_US = 60000;                 // correccion mas vieja que esto no se aplica

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- lo escribe loop() (nucleo 1), siempre dentro de g_mux
static ekf5b::Param g_param;
static volatile int  g_modo = APAGADO;
static volatile bool g_permitido = false;      // el estado del robot admite correccion
static volatile bool g_pedirReinicio = true;   // tras SETHOME, STOP, JOG, cambio de modo...

// ---- lo escribe la tarea de la IMU (nucleo 0)
static volatile float    g_dq1 = 0.0f, g_dq2 = 0.0f;   // correccion a sumar a los pulsos (rad)
static volatile uint32_t g_tUltima = 0;                // micros() de la ultima correccion calculada
static volatile int      g_evento = EV_NINGUNO;
static volatile bool     g_ekfOk = false;
static volatile uint32_t g_durUlt = 0, g_durMax = 0;   // tiempo de calculo del EKF + ley (us)

inline void anotarDuracion(uint32_t us) {
    g_durUlt = us;
    if (us > g_durMax) g_durMax = us;
}

inline const char *nombreEvento(int e) {
    switch (e) {
        case EV_IMU_INVALIDA: return "IMU_INVALIDA";
        case EV_INNOVACION:   return "INNOVACION";
        case EV_EKF:          return "EKF";
        case EV_SIN_LECTURA:  return "SIN_LECTURA";
        default:              return "NINGUNO";
    }
}

// ---------------------------------------------------------------- plan (nucleo 1 -> nucleo 0)
struct PtoPlan { uint32_t t; float q1, q2, x, y; };
const int NPLAN = 128;                          // 0.5 s a 250 Hz
static PtoPlan g_plan[NPLAN];
static volatile uint32_t g_planN = 0;

// t: instante (micros) en que el plan debe estar en (q1, q2); (x, y) en metros
inline void registrarPlan(uint32_t t, float q1, float q2, float x, float y) {
    portENTER_CRITICAL(&g_mux);
    g_plan[g_planN % NPLAN] = {t, q1, q2, x, y};
    g_planN = g_planN + 1;
    portEXIT_CRITICAL(&g_mux);
}

// Plan interpolado en el instante t.  false si no hay plan para ese instante.
inline bool planEn(uint32_t t, float q[2], float &x, float &y) {
    bool ok = false;
    portENTER_CRITICAL(&g_mux);
    uint32_t n = g_planN;
    int disp = n < (uint32_t)NPLAN ? (int)n : NPLAN;
    if (disp >= 2) {
        const PtoPlan &nuevo = g_plan[(n - 1) % NPLAN];
        if ((int32_t)(t - nuevo.t) >= 0) {             // despues del ultimo punto: se mantiene
            if ((int32_t)(t - nuevo.t) < 50000) { q[0] = nuevo.q1; q[1] = nuevo.q2; x = nuevo.x; y = nuevo.y; ok = true; }
        } else {
            for (int k = 1; k < disp; k++) {
                const PtoPlan &b = g_plan[(n - k) % NPLAN];
                const PtoPlan &a = g_plan[(n - k - 1) % NPLAN];
                if ((int32_t)(t - a.t) >= 0 && (int32_t)(b.t - t) > 0) {
                    float u = (float)(int32_t)(t - a.t) / (float)(int32_t)(b.t - a.t);
                    q[0] = a.q1 + u * (b.q1 - a.q1);
                    q[1] = a.q2 + u * (b.q2 - a.q2);
                    x = a.x + u * (b.x - a.x);
                    y = a.y + u * (b.y - a.y);
                    ok = true;
                    break;
                }
            }
        }
    }
    portEXIT_CRITICAL(&g_mux);
    return ok;
}

// Correccion que loop() debe sumar ahora (rad).  0 si no hay una reciente o hay un reinicio pendiente.
inline void dqActual(float &a, float &b) {
    portENTER_CRITICAL(&g_mux);
    bool vigente = !g_pedirReinicio && (uint32_t)(micros() - g_tUltima) < VIGENCIA_US;
    a = vigente ? g_dq1 : 0.0f;
    b = vigente ? g_dq2 : 0.0f;
    portEXIT_CRITICAL(&g_mux);
}

inline void pedirReinicio() {
    portENTER_CRITICAL(&g_mux);
    g_pedirReinicio = true;
    portEXIT_CRITICAL(&g_mux);
}

inline void ponerPermitido(bool p) {
    portENTER_CRITICAL(&g_mux);
    g_permitido = p;
    portEXIT_CRITICAL(&g_mux);
}

// Lee y borra el ultimo evento de seguridad (lo reporta loop()).
inline int tomarEvento() {
    portENTER_CRITICAL(&g_mux);
    int e = g_evento;
    g_evento = EV_NINGUNO;
    portEXIT_CRITICAL(&g_mux);
    return e;
}

// ---------------------------------------------------------------- tarea de la IMU (nucleo 0)
struct Salida {
    float q1h = 0, q2h = 0, w1h = 0, w2h = 0, d1 = 0, d2 = 0, dl1 = 0, dl2 = 0, inov = 0;
    uint8_t estado = 0;                         // bit0-1: modo; bit2: EKF valido; bit3: aplicando
};

namespace detalle {
struct Hist { uint32_t t; float q1, q2; };
const int NH = 10;
static Hist h[NH];
static int hn = 0;
static ekf5b::EKF ekf;
static ekf5b::Correccion cor;
static ekf5b::Entrada uPrev;
static uint32_t tPrev = 0;
static int malas = 0, inovAlta = 0, sinLeer = 0;
static float aplicado[2] = {0, 0};
static int modoPrevio = APAGADO;

// angulo emitido interpolado en t (de la historia de las muestras)
inline bool emitidoEn(uint32_t t, float q[2]) {
    int n = hn < NH ? hn : NH;
    for (int k = 0; k + 1 < n; k++) {
        const Hist &b = h[(hn - 1 - k) % NH];
        const Hist &a = h[(hn - 2 - k) % NH];
        if ((int32_t)(t - a.t) >= 0 && (int32_t)(b.t - t) >= 0) {
            int32_t den = (int32_t)(b.t - a.t);
            float u = den > 0 ? (float)(int32_t)(t - a.t) / (float)den : 0.0f;
            q[0] = a.q1 + u * (b.q1 - a.q1);
            q[1] = a.q2 + u * (b.q2 - a.q2);
            return true;
        }
    }
    return false;
}

inline void rampaACero(float paso) {
    for (int i = 0; i < 2; i++) {
        if (aplicado[i] > paso) aplicado[i] -= paso;
        else if (aplicado[i] < -paso) aplicado[i] += paso;
        else aplicado[i] = 0.0f;
    }
}

// publica la correccion y, si hubo un evento con la correccion activa, pasa a SOMBRA
inline void publicar(uint32_t t_us, int evento) {
    portENTER_CRITICAL(&g_mux);
    g_dq1 = aplicado[0];
    g_dq2 = aplicado[1];
    g_tUltima = t_us;
    if (evento != EV_NINGUNO && g_modo == ACTIVO) {
        g_modo = SOMBRA;
        if (g_evento == EV_NINGUNO) g_evento = evento;
    }
    portEXIT_CRITICAL(&g_mux);
}
}  // namespace detalle

// La lectura de la IMU fallo (I2C): tras N_SEGUIDAS fallos seguidos la correccion se apaga.
inline void sinLectura() {
    using namespace detalle;
    ekf5b::Param p;
    portENTER_CRITICAL(&g_mux);
    p = g_param;
    portEXIT_CRITICAL(&g_mux);
    int ev = (++sinLeer >= N_SEGUIDAS) ? EV_SIN_LECTURA : EV_NINGUNO;
    rampaACero(p.dpaso);
    publicar(micros(), ev);
}

// q1, q2: angulos emitidos (rad) en el instante t_us de la lectura.  gz [rad/s], ax, ay [m/s²].
inline void procesar(uint32_t t_us, float q1, float q2, float gz, float ax, float ay, bool valida, Salida &s) {
    using namespace detalle;
    ekf5b::Param p;
    int modo;
    bool permitido, reinicio;
    portENTER_CRITICAL(&g_mux);
    p = g_param;
    modo = g_modo;
    permitido = g_permitido;
    reinicio = g_pedirReinicio;
    g_pedirReinicio = false;
    portEXIT_CRITICAL(&g_mux);
    sinLeer = 0;

    // reinicio (SETHOME, STOP, JOG...): los pulsos pueden haber saltado; se descarta la historia
    if (reinicio) {
        hn = 0;
        aplicado[0] = aplicado[1] = 0.0f;
        ekf.iniciado = false;
    }
    h[hn % NH] = {t_us, q1, q2};
    hn++;
    s.q1h = q1; s.q2h = q2;

    if (modo == APAGADO) {                       // no corre el EKF; si venia aplicando, rampa a cero
        rampaACero(p.dpaso);
        modoPrevio = APAGADO;
        g_ekfOk = false;
        ekf.iniciado = false;
        publicar(t_us, EV_NINGUNO);
        s.dl1 = aplicado[0]; s.dl2 = aplicado[1];
        s.estado = (aplicado[0] != 0.0f || aplicado[1] != 0.0f) ? 8 : 0;
        return;
    }
    if (modoPrevio == APAGADO) ekf.iniciado = false;
    modoPrevio = modo;

    // ---- entrada del modelo: angulos emitidos en t − tau y su derivada (±20 ms)
    uint32_t tr = t_us - (uint32_t)(p.tau * 1e6f);
    float r[2], ra[2], rb[2];
    if (!emitidoEn(tr, r) || !emitidoEn(tr - 20000, ra) || !emitidoEn(tr + 20000, rb)) {
        rampaACero(p.dpaso);                     // todavia no hay historia suficiente
        publicar(t_us, EV_NINGUNO);
        s.estado = (uint8_t)modo;
        return;
    }
    ekf5b::Entrada u{{r[0], r[1]}, {(rb[0] - ra[0]) / 0.04f, (rb[1] - ra[1]) / 0.04f}};

    if (!ekf.iniciado) {
        ekf.p = p;
        ekf.reiniciar(r[0], r[1]);
        cor.reiniciar();
        uPrev = u;
        tPrev = t_us;
        malas = inovAlta = 0;
    }
    ekf.p = p;
    float dt = (int32_t)(t_us - tPrev) * 1e-6f;
    if (dt < 0 || dt > 0.05f) {                  // hueco largo: se reinicia el filtro
        ekf.reiniciar(r[0], r[1]);
        cor.reiniciar();
        uPrev = u;
        dt = 0;
    }
    float z[3] = {gz, ax, ay};
    bool ok = ekf.paso(dt, uPrev, u, z, valida, true);
    uPrev = u;
    tPrev = t_us;

    int evento = EV_NINGUNO;
    if (!ok) {
        ekf.reiniciar(r[0], r[1]);
        cor.reiniciar();
        evento = EV_EKF;
    }
    if (!valida) { if (++malas >= N_SEGUIDAS) evento = EV_IMU_INVALIDA; }
    else {
        malas = 0;
        if (fabsf(ekf.inov[0]) > INOV_MAX) { if (++inovAlta >= N_SEGUIDAS) evento = EV_INNOVACION; }
        else inovAlta = 0;
    }
    g_ekfOk = ok;

    // ---- ley de correccion (se calcula tambien en SOMBRA, para registrarla)
    float qp[2], xp, yp, qa[2], qb[2], xa, ya;
    bool hayPlan = planEn(tr, qp, xp, yp) && planEn(tr - 8000, qa, xa, ya) && planEn(tr + 8000, qb, xa, ya);
    bool aplicar = ok && hayPlan && permitido && modo == ACTIVO && evento == EV_NINGUNO;
    if (ok && hayPlan) {
        float vp[2] = {(qb[0] - qa[0]) / 0.016f, (qb[1] - qa[1]) / 0.016f};
        cor.calcular(p, ekf, qp, vp, xp, yp, aplicar, dt > 0 ? dt : 0.01f);
    }
    if (aplicar) { aplicado[0] = cor.delta[0]; aplicado[1] = cor.delta[1]; }
    else rampaACero(p.dpaso);
    publicar(t_us, evento);

    s.q1h = ekf.x[0]; s.q2h = ekf.x[1];
    s.w1h = ekf.x[2]; s.w2h = ekf.x[3];
    s.d1 = ekf.x[4]; s.d2 = ekf.x[5];
    s.dl1 = cor.delta[0]; s.dl2 = cor.delta[1];
    s.inov = ekf.inov[0];
    s.estado = (uint8_t)(modo | (ok ? 4 : 0) | (aplicar ? 8 : 0));
}

// ---------------------------------------------------------------- comandos (nucleo 1)
inline void reportar() {
    ekf5b::Param p;
    int modo;
    portENTER_CRITICAL(&g_mux);
    p = g_param;
    modo = g_modo;
    portEXIT_CRITICAL(&g_mux);
    const float G = 57.29578f;
    Serial.printf("CTL modo=%d ley=%s alfa=%.3f zona=%.2f dmax=%.3f lavado=%.2f ekf=%d t_ekf=%lu/%lu\n", modo,
                  p.ley == 1 ? "LQR" : "ALFA", p.alfa, p.zona * G, p.dmax * G, p.lavado, g_ekfOk ? 1 : 0,
                  (unsigned long)g_durUlt, (unsigned long)g_durMax);
}

// Atiende "CTL ..." (el primer token ya fue leido con strtok).  imuOk: la BNO055 esta funcionando.
//   CTL                          estado
//   CTL MODO 0|1|2               apagado / sombra / activo   (tambien APAGADO, SOMBRA, ACTIVO)
//   CTL LEY ALFA|LQR
//   CTL ALFA v                   amortiguamiento agregado (fraccion del critico, 0..1)
//   CTL ZONA gps                 zona muerta del error de velocidad (grados/s)
//   CTL DMAX g                   recorte de la correccion (grados, 0..3)
//   CTL LAVADO s                 constante del filtro de lavado (s, 0 = sin filtro)
//   CTL GRAV cx cy p1 p2         ajuste de gravedad del acelerometro
//   CTL RESET                    reinicia el EKF
inline void comando(bool imuOk) {
    char *a = strtok(NULL, " \t\r\n");
    if (!a) { reportar(); return; }
    const float R = 0.017453293f;
    if (!strcasecmp(a, "MODO")) {
        char *b = strtok(NULL, " \t\r\n");
        if (!b) { Serial.println("ERR ctl modo 0|1|2"); return; }
        int m = !strcasecmp(b, "APAGADO") ? 0 : !strcasecmp(b, "SOMBRA") ? 1 : !strcasecmp(b, "ACTIVO") ? 2 : atoi(b);
        if (m < 0 || m > 2) { Serial.println("ERR ctl modo 0|1|2"); return; }
        if (m > 0 && !imuOk) { Serial.println("ERR ctl: la IMU no esta funcionando"); return; }
        portENTER_CRITICAL(&g_mux);
        g_modo = m;
        g_evento = EV_NINGUNO;
        portEXIT_CRITICAL(&g_mux);
        Serial.printf("OK ctl modo %d\n", m);
    } else if (!strcasecmp(a, "LEY")) {
        char *b = strtok(NULL, " \t\r\n");
        int l = (b && !strcasecmp(b, "LQR")) ? 1 : (b && !strcasecmp(b, "ALFA")) ? 0 : -1;
        if (l < 0) { Serial.println("ERR ctl ley ALFA|LQR"); return; }
        portENTER_CRITICAL(&g_mux);
        g_param.ley = l;
        portEXIT_CRITICAL(&g_mux);
        pedirReinicio();
        Serial.printf("OK ctl ley %s\n", l ? "LQR" : "ALFA");
    } else if (!strcasecmp(a, "ALFA") || !strcasecmp(a, "ZONA") || !strcasecmp(a, "DMAX") || !strcasecmp(a, "LAVADO")) {
        char *b = strtok(NULL, " \t\r\n");
        if (!b) { Serial.printf("ERR ctl %s <valor>\n", a); return; }
        float v = atof(b);
        bool bien = true;
        portENTER_CRITICAL(&g_mux);
        if (!strcasecmp(a, "ALFA"))      { if (v >= 0 && v <= 1.0f) g_param.alfa = v; else bien = false; }
        else if (!strcasecmp(a, "ZONA")) { if (v >= 0 && v <= 20.0f) g_param.zona = v * R; else bien = false; }
        else if (!strcasecmp(a, "DMAX")) { if (v >= 0 && v <= 3.0f) g_param.dmax = v * R; else bien = false; }
        else                             { if (v >= 0 && v <= 10.0f) g_param.lavado = v; else bien = false; }
        portEXIT_CRITICAL(&g_mux);
        if (!bien) { Serial.printf("ERR ctl %s fuera de rango\n", a); return; }
        Serial.printf("OK ctl %s %.3f\n", a, v);
    } else if (!strcasecmp(a, "GRAV")) {
        float v[4];
        for (int i = 0; i < 4; i++) {
            char *b = strtok(NULL, " \t\r\n");
            if (!b) { Serial.println("ERR ctl grav cx cy p1 p2"); return; }
            v[i] = atof(b);
        }
        portENTER_CRITICAL(&g_mux);
        g_param.gcx = v[0]; g_param.gcy = v[1]; g_param.gp1 = v[2]; g_param.gp2 = v[3];
        portEXIT_CRITICAL(&g_mux);
        pedirReinicio();
        Serial.println("OK ctl grav");
    } else if (!strcasecmp(a, "RESET")) {
        pedirReinicio();
        Serial.println("OK ctl reset");
    } else {
        Serial.printf("ERR ctl: %s?\n", a);
        return;
    }
    reportar();
}

}  // namespace ctl
#endif  // CONTROL_IMU_H

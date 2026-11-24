#ifndef KINEMATICS_5BAR_H
#define KINEMATICS_5BAR_H
// ============================================================================
//  NUCLEO CINEMATICO - Robot Paralelo 5 Barras
//  Header puro (solo <cmath>), compartido por el firmware y los tests.
//  UNICA FUENTE DE VERDAD para geometria + conversiones.
//
//  MODOS DE ENSAMBLAJE (codos) -- misma numeracion que ik5bar.m (MATLAB):
//     modo 1: sgn = [+1, -1]   codos AFUERA (izq. abre a la izq., der. a la der.)
//     modo 2: sgn = [-1, +1]   codos ADENTRO
//     modo 3: sgn = [+1, +1]   codo izq. afuera, codo der. adentro
//     modo 4: sgn = [-1, -1]   codo izq. adentro, codo der. afuera
//  th_i = phi_i + sgn_i * beta_i.  El modo activo se elige en tiempo de
//  ejecucion (comando MODE).  El utillaje del cero define el modo fisico.
// ============================================================================
#include <cmath>

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

namespace rk {   // robot kinematics

// ---- GEOMETRIA (m) -- debe coincidir con tu robot fisico ----
// Medidas actualizadas (oct. 2026): proximales 120 mm, distales 180 mm, D = 150 mm
constexpr float L1 = 0.120f;   // proximal izquierdo  (en el motor O1)
constexpr float L2 = 0.180f;   // distal izquierdo
constexpr float L3 = 0.180f;   // distal derecho
constexpr float L4 = 0.120f;   // proximal derecho    (en el motor O2)
constexpr float D  = 0.150f;   // separacion de bases O1-O2 (centro a centro de ejes)

// ---- RESOLUCION DE LOS SERVOS (pulsos por revolucion) ----
// DEBE coincidir EXACTAMENTE con el valor configurado en el iHSV57
// (DIP SW1-SW4 o software de JMC).  Configuracion actual de los servos: 8000.
// Si cambian los interruptores del driver, cambien tambien este valor y
// vuelvan a fijar el cero.
constexpr float STEPS_PER_REV = 8000.0f;
constexpr float STEPS_PER_RAD = STEPS_PER_REV / (2.0f * PI_F);

// ---- GUARDA DE SINGULARIDAD ----
// |sin(a1 - a2)| ~ |det(J_P)|.  Cae a 0 cuando los eslabones distales
// se alinean (singularidad paralela / tipo 2).  Margen sano > 0.15.
constexpr float SIN_MIN = 0.15f;

constexpr float TOL = 1e-6f;

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Signos del modo de ensamblaje (1..4).  false si el modo no existe.
inline bool modeSigns(int mode, int &s1, int &s2) {
    switch (mode) {
        case 1: s1 = +1; s2 = -1; return true;
        case 2: s1 = -1; s2 = +1; return true;
        case 3: s1 = +1; s2 = +1; return true;
        case 4: s1 = -1; s2 = -1; return true;
        default: return false;
    }
}

// Angulo equivalente (th + k*2pi) mas cercano a ref.  Evita saltos de 360 deg.
inline float wrapNear(float th, float ref) {
    return ref + remainderf(th - ref, 2.0f * PI_F);
}

// ---------------------------------------------------------------------------
//  CINEMATICA INVERSA en el modo dado.
//  Devuelve true si (x,y) es alcanzable.  th1, th2 en radianes (sin envolver).
// ---------------------------------------------------------------------------
inline bool ik5bar(float x, float y, int mode, float &th1, float &th2) {
    int s1, s2;
    if (!modeSigns(mode, s1, s2)) return false;
    // Cadena izquierda: O1=(0,0)
    float r1 = sqrtf(x * x + y * y);
    if (r1 < TOL || r1 > L1 + L2 + TOL || r1 < fabsf(L1 - L2) - TOL) return false;
    float phi1 = atan2f(y, x);
    float c1   = clampf((r1 * r1 + L1 * L1 - L2 * L2) / (2.0f * r1 * L1), -1.0f, 1.0f);
    float b1   = acosf(c1);

    // Cadena derecha: O2=(D,0)
    float dx = x - D;
    float r2 = sqrtf(dx * dx + y * y);
    if (r2 < TOL || r2 > L4 + L3 + TOL || r2 < fabsf(L4 - L3) - TOL) return false;
    float phi2 = atan2f(y, dx);
    float c2   = clampf((r2 * r2 + L4 * L4 - L3 * L3) / (2.0f * r2 * L4), -1.0f, 1.0f);
    float b2   = acosf(c2);

    th1 = phi1 + s1 * b1;
    th2 = phi2 + s2 * b2;
    return true;
}

// ---------------------------------------------------------------------------
//  CINEMATICA DIRECTA: las DOS soluciones (interseccion de dos circunferencias)
//  rama 0 = "+h" (a la izquierda de A->B; arriba cuando A esta a la izq. de B)
//  rama 1 = "-h"
// ---------------------------------------------------------------------------
inline bool fk5barBoth(float th1, float th2, float x[2], float y[2]) {
    float Ax = L1 * cosf(th1), Ay = L1 * sinf(th1);
    float Bx = D + L4 * cosf(th2), By = L4 * sinf(th2);
    float ex = Bx - Ax, ey = By - Ay;
    float dAB = sqrtf(ex * ex + ey * ey);
    if (dAB > (L2 + L3) + TOL || dAB < fabsf(L2 - L3) - TOL || dAB < TOL) return false;
    float a = (L2 * L2 - L3 * L3 + dAB * dAB) / (2.0f * dAB);
    float h2 = L2 * L2 - a * a;
    float h = sqrtf(h2 < 0.0f ? 0.0f : h2);
    float Mx = Ax + a * ex / dAB, My = Ay + a * ey / dAB;
    float px = -ey / dAB, py = ex / dAB;     // perpendicular unitaria
    x[0] = Mx + h * px;  y[0] = My + h * py;
    x[1] = Mx - h * px;  y[1] = My - h * py;
    return true;
}

// FK eligiendo la solucion MAS CERCANA a (xref,yref): sigue la rama fisica
// por continuidad (el efector no salta de rama sin cruzar una singularidad).
inline bool fk5barNear(float th1, float th2, float xref, float yref, float &x, float &y) {
    float xs[2], ys[2];
    if (!fk5barBoth(th1, th2, xs, ys)) return false;
    float d0 = (xs[0]-xref)*(xs[0]-xref) + (ys[0]-yref)*(ys[0]-yref);
    float d1 = (xs[1]-xref)*(xs[1]-xref) + (ys[1]-yref)*(ys[1]-yref);
    int k = (d0 <= d1) ? 0 : 1;
    x = xs[k]; y = ys[k];
    return true;
}

// true si los angulos (th1,th2) son la solucion IK del modo en el punto (x,y)
inline bool anglesMatchMode(float x, float y, float th1, float th2, int mode, float tolRad) {
    float i1, i2;
    if (!ik5bar(x, y, mode, i1, i2)) return false;
    return fabsf(remainderf(i1 - th1, 2.0f * PI_F)) < tolRad &&
           fabsf(remainderf(i2 - th2, 2.0f * PI_F)) < tolRad;
}

// FK coherente con un modo: devuelve la rama cuyo punto, resuelto con la IK de
// ese modo, reproduce los angulos.  false si ninguna rama corresponde al modo.
inline bool fk5barMode(float th1, float th2, int mode, float &x, float &y) {
    float xs[2], ys[2];
    if (!fk5barBoth(th1, th2, xs, ys)) return false;
    for (int k = 0; k < 2; k++) {
        if (anglesMatchMode(xs[k], ys[k], th1, th2, mode, 1e-3f)) { x = xs[k]; y = ys[k]; return true; }
    }
    return false;
}

// Modo de ensamblaje que corresponde a un punto y unos angulos (0 si ninguno)
inline int detectMode(float x, float y, float th1, float th2) {
    for (int m = 1; m <= 4; m++)
        if (anglesMatchMode(x, y, th1, th2, m, 1e-3f)) return m;
    return 0;
}

// ---------------------------------------------------------------------------
//  MEDIDA DE SINGULARIDAD a partir del punto y los angulos.
// ---------------------------------------------------------------------------
inline float singularityMeasure(float x, float y, float th1, float th2) {
    float Ax = L1 * cosf(th1), Ay = L1 * sinf(th1);
    float Bx = D + L4 * cosf(th2), By = L4 * sinf(th2);
    float a1 = atan2f(y - Ay, x - Ax);
    float a2 = atan2f(y - By, x - Bx);
    return fabsf(sinf(a1 - a2));
}

inline bool isReachableAndSafe(float x, float y, int mode) {
    float t1, t2;
    if (!ik5bar(x, y, mode, t1, t2)) return false;
    return singularityMeasure(x, y, t1, t2) >= SIN_MIN;
}

// ---------------------------------------------------------------------------
//  PERFIL QUINTICO  s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5,  tau in [0,1]
//  s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
// ---------------------------------------------------------------------------
inline float quinticS(float tau) {
    if (tau <= 0.0f) return 0.0f;
    if (tau >= 1.0f) return 1.0f;
    float t3 = tau * tau * tau;
    return t3 * (10.0f - 15.0f * tau + 6.0f * tau * tau);
}

// ---------------------------------------------------------------------------
//  Duracion minima de un segmento respetando Vmax y Amax (perfil quintico).
//  pico v = 1.875 D / T   ;   pico a = (10/sqrt3) D / T^2 = 5.7735 D / T^2
// ---------------------------------------------------------------------------
inline float segDuration(float dist, float vmax, float amax) {
    if (dist < TOL) return 0.05f;
    float Tv = 1.875f * dist / vmax;
    float Ta = sqrtf(5.7735f * dist / amax);
    return Tv > Ta ? Tv : Ta;
}

// ---------------------------------------------------------------------------
//  PERFIL CONTINUO PARA TRAZOS (lo usa PATH/GOTO)
//  Un "trazo" es un tramo de la figura que se recorre SIN detenerse (p.ej. un
//  circulo completo, o un lado de un cuadrado).  Perfil de velocidad:
//    - rampa de aceleracion suave: v(t) = vc * quinticS(t/Ta)
//      (la aceleracion arranca y termina en 0 -> sin tirones)
//    - CRUCERO a velocidad constante vc
//    - frenado simetrico
//  pico de acel en la rampa = 1.875 vc / Ta  ->  Ta = 1.875 vc / amax
//  distancia de cada rampa  = vc Ta / 2
//  Si el trazo es corto y no da para llegar a vmax, se reduce vc.
// ---------------------------------------------------------------------------
struct SmoothProfile {
    float L;    // longitud del trazo (m)
    float vc;   // velocidad de crucero (m/s)
    float Ta;   // duracion de cada rampa (s)
    float Tc;   // duracion del crucero (s)
    float T;    // duracion total (s)
};

// Integral de quinticS:  Q(tau) = 2.5 tau^4 - 3 tau^5 + tau^6 ,  Q(1) = 0.5
inline float quinticInt(float tau) {
    if (tau <= 0.0f) return 0.0f;
    if (tau >= 1.0f) return 0.5f;
    float t4 = tau * tau * tau * tau;
    return t4 * (2.5f - 3.0f * tau + tau * tau);
}

inline SmoothProfile makeProfile(float L, float vmax, float amax) {
    SmoothProfile p = {L, 0.0f, 0.0f, 0.0f, 0.0f};
    if (L < TOL || vmax <= 0.0f || amax <= 0.0f) return p;
    float vc = vmax;
    float Ta = 1.875f * vc / amax;
    if (vc * Ta > L) {                    // no hay espacio para crucero
        vc = sqrtf(L * amax / 1.875f);
        Ta = 1.875f * vc / amax;
    }
    p.vc = vc;
    p.Ta = Ta;
    p.Tc = (L - vc * Ta) / vc;
    if (p.Tc < 0.0f) p.Tc = 0.0f;
    p.T  = 2.0f * Ta + p.Tc;
    return p;
}

// Distancia recorrida sobre el trazo en el instante t (0 <= s <= L)
inline float profilePos(const SmoothProfile &p, float t) {
    if (t <= 0.0f)  return 0.0f;
    if (t >= p.T)   return p.L;
    if (t < p.Ta)   return p.vc * p.Ta * quinticInt(t / p.Ta);
    if (t < p.Ta + p.Tc) return 0.5f * p.vc * p.Ta + p.vc * (t - p.Ta);
    return p.L - p.vc * p.Ta * quinticInt((p.T - t) / p.Ta);
}

inline long radToSteps(float th) {
    return lroundf(th * STEPS_PER_RAD);
}

} // namespace rk
#endif // KINEMATICS_5BAR_H

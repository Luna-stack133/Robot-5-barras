#ifndef KINEMATICS_5BAR_H
#define KINEMATICS_5BAR_H
// ============================================================================
//  NUCLEO CINEMATICO - Robot Paralelo 5 Barras
//  Header puro (solo <cmath>), compartido por el firmware y los tests.
//  UNICA FUENTE DE VERDAD para geometria + conversiones.
//
//  Convencion de ensamblaje: IK config 3  (sgn = [+1, +1]).
//  El homing fisico DEBE dejar el robot en este mismo modo de ensamblaje.
// ============================================================================
#include <cmath>

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

namespace rk {   // robot kinematics

// ---- GEOMETRIA (m) -- debe coincidir con tu robot fisico ----
constexpr float L1 = 0.090f;   // proximal izquierdo  (en el motor O1)
constexpr float L2 = 0.165f;   // distal izquierdo
constexpr float L3 = 0.165f;   // distal derecho
constexpr float L4 = 0.090f;   // proximal derecho    (en el motor O2)
constexpr float D  = 0.155f;   // separacion de bases O1-O2

// ---- RESOLUCION DE LOS SERVOS (pulsos por revolucion) ----
// DEBE coincidir EXACTAMENTE con el valor configurado en el iHSV57
// (DIP SW1-SW4 o software de JMC).  Recomendado: 8000.
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

// ---------------------------------------------------------------------------
//  CINEMATICA INVERSA  (config 3)
//  Devuelve true si (x,y) es alcanzable.  th1, th2 en radianes (sin envolver).
// ---------------------------------------------------------------------------
inline bool ik5bar(float x, float y, float &th1, float &th2) {
    // Cadena izquierda: O1=(0,0)
    float r1 = sqrtf(x * x + y * y);
    if (r1 > L1 + L2 + TOL || r1 < fabsf(L1 - L2) - TOL) return false;
    float phi1 = atan2f(y, x);
    float c1   = clampf((r1 * r1 + L1 * L1 - L2 * L2) / (2.0f * r1 * L1), -1.0f, 1.0f);
    float b1   = acosf(c1);

    // Cadena derecha: O2=(D,0)
    float dx = x - D;
    float r2 = sqrtf(dx * dx + y * y);
    if (r2 > L4 + L3 + TOL || r2 < fabsf(L4 - L3) - TOL) return false;
    float phi2 = atan2f(y, dx);
    float c2   = clampf((r2 * r2 + L4 * L4 - L3 * L3) / (2.0f * r2 * L4), -1.0f, 1.0f);
    float b2   = acosf(c2);

    // config 3 -> sgn = [+1, +1]
    th1 = phi1 + b1;
    th2 = phi2 + b2;
    return true;
}

// ---------------------------------------------------------------------------
//  CINEMATICA DIRECTA  (config 1, que es la rama que corresponde a IK config 3)
//  Interseccion de dos circunferencias.  Devuelve true si es valida.
// ---------------------------------------------------------------------------
inline bool fk5bar(float th1, float th2, float &x, float &y) {
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
    x = Mx + h * px;                          // config 1 (+)
    y = My + h * py;
    return true;
}

// ---------------------------------------------------------------------------
//  MEDIDA DE SINGULARIDAD a partir del punto comandado y los angulos.
//  (No requiere FK: usa la posicion objetivo directamente.)
// ---------------------------------------------------------------------------
inline float singularityMeasure(float x, float y, float th1, float th2) {
    float Ax = L1 * cosf(th1), Ay = L1 * sinf(th1);
    float Bx = D + L4 * cosf(th2), By = L4 * sinf(th2);
    float a1 = atan2f(y - Ay, x - Ax);
    float a2 = atan2f(y - By, x - Bx);
    return fabsf(sinf(a1 - a2));
}

inline bool isReachableAndSafe(float x, float y) {
    float t1, t2;
    if (!ik5bar(x, y, t1, t2)) return false;
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

inline long radToSteps(float th) {
    return lroundf(th * STEPS_PER_RAD);
}

} // namespace rk
#endif // KINEMATICS_5BAR_H

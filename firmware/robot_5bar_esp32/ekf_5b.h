// ============================================================================
//  ekf_5b.h  -  EKF del robot de 5 barras con la IMU del distal derecho, y la
//               ley de correccion de la referencia (firmware 1.3, etapa 4).
//
//  Es la misma formulacion que etapa2/ekf5b.py (validada contra el video) y que
//  la simulacion etapa2/lazo_sim.py, pero con la cinematica de velocidades y
//  aceleraciones ANALITICA (lazo cerrado del 5 barras) en lugar de diferencias
//  finitas: asi todo funciona en float (la ESP32-S3 solo tiene FPU de float).
//
//  Estados (7):  q1, q2 [rad] · w1, w2 [rad/s] · d1, d2 [N·m] · b [rad/s]
//  Modelo:       M(q)·w' = K·(r − q) + B·(ff·r' − w) − C(q,w)·w + d
//                r = angulos emitidos (pulsos) con el retardo tau del servo
//                d' = −d/tau_d + ruido   (par de perturbacion de Gauss-Markov)
//  Mediciones:   gz = k_g·φ3' + b
//                ax, ay = aceleracion del chip en ejes del eslabon + gravedad
//
//  Ley de correccion (Δq, se suma a los pulsos):
//    ALFA: Δq = −K⁻¹·D·zona(ŵ − q̇_plan(t−tau)),  D_i = 2·α·√(K_i·M_ii)
//    LQR : Δq = −F(x,y)·[q̂ − q_plan, ŵ − q̇_plan, Δ(k−1) … Δ(k−4)]
//          F de la tabla lqr_tabla.h (modelo con el retardo, Riccati discreto)
//    Recorte |Δq| ≤ dmax y limite de cambio por paso.
//
//  Este archivo NO depende de Arduino: se compila igual en el PC para probarlo
//  contra los datos grabados (ver prueba_pc/).
// ============================================================================
#ifndef EKF_5B_H
#define EKF_5B_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "lqr_tabla.h"

namespace ekf5b {

// ---------------------------------------------------------------- geometria (m)
const float GL1 = 0.120f, GL2 = 0.180f, GL3 = 0.180f, GL4 = 0.120f, GD = 0.150f;
// ---------------------------------------------------------------- masas (kg) e inercias
const float M_PROX = 0.115f, M_DIST = 0.170f, M_EFEC = 0.334f;
const float CM_PROX = 0.045f, CM_DIST = 0.090f;
const float J_ROTOR = 0.3e-4f;               // kg·m² (P00-04 = 0.3 kg·cm²)
const float R_IMU = 0.0335f;                 // m: del eje del efector al chip, sobre el distal derecho
const float K_GIRO = 1.031f;                 // escala del giroscopio
const float G0 = 9.80665f;

// ---------------------------------------------------------------- parametros ajustables
struct Param {
    // modelo del servo (identificado el 13 nov 2026 con la sintonia congelada)
    float K1 = 1.33455f, K2 = 0.86507f;      // N·m/rad
    float B1 = 0.035686f, B2 = 0.066359f;    // N·m·s/rad
    float ff = 0.60900f;                     // feedforward de velocidad efectivo
    float tau = 0.02338f;                    // s
    // gravedad en los ejes del chip segun el angulo del distal (ajuste con 36 reposos)
    float gcx = 0.98766f, gcy = -0.22155f, gp1 = -0.10295f, gp2 = 0.26579f;
    float psi = -177.0f * 0.017453293f;      // montaje del chip (rad)
    // ruido del EKF
    float s_q = 1e-5f, s_w = 0.05f, sd_d = 0.03f, tau_d = 0.3f, s_b = 0.002f;
    float r_gz = 0.8f * 0.017453293f, r_acc = 0.35f;
    // ley de correccion
    int   ley = 0;                           // 0 = ALFA, 1 = LQR
    float alfa = 0.3f;
    float zona = 1.0f * 0.017453293f;        // rad/s
    float dmax = 1.0f * 0.017453293f;        // rad
    float dpaso = 0.3f * 0.017453293f;       // cambio maximo de Δq por muestra (rad)
    float lavado = 1.0f;                     // s: filtro de lavado de la correccion (0 = sin filtro).
                                             // La correccion no puede sostener un valor constante: un sesgo
                                             // del acelerometro no se convierte en error de posicion en reposo.
};

// ---------------------------------------------------------------- cinematica
struct Cin {
    bool  ok;
    float q1, q2;
    float A[2], B[2], P[2];
    float e2[2], e3[2];                      // unitarios de los distales (A→P, B→P)
    float phi2, phi3;
    float G[2][2];                           // [φ2'; φ3'] = G·q'
};

inline bool cinematica(float q1, float q2, Cin &c) {
    c.q1 = q1; c.q2 = q2;
    float c1 = cosf(q1), s1 = sinf(q1), c2 = cosf(q2), s2 = sinf(q2);
    c.A[0] = GL1 * c1; c.A[1] = GL1 * s1;
    c.B[0] = GD + GL4 * c2; c.B[1] = GL4 * s2;
    float dx = c.B[0] - c.A[0], dy = c.B[1] - c.A[1];
    float d = sqrtf(dx * dx + dy * dy);
    if (d < 1e-6f || d > GL2 + GL3) { c.ok = false; return false; }
    float a = (GL2 * GL2 - GL3 * GL3 + d * d) / (2.0f * d);
    float h2 = GL2 * GL2 - a * a;
    float h = h2 > 0 ? sqrtf(h2) : 0.0f;
    float mx = c.A[0] + a * dx / d, my = c.A[1] + a * dy / d;
    float nx = -dy / d, ny = dx / d;
    float p1x = mx + h * nx, p1y = my + h * ny, p2x = mx - h * nx, p2y = my - h * ny;
    if (p1y >= p2y) { c.P[0] = p1x; c.P[1] = p1y; } else { c.P[0] = p2x; c.P[1] = p2y; }   // efector arriba
    c.e2[0] = (c.P[0] - c.A[0]) / GL2; c.e2[1] = (c.P[1] - c.A[1]) / GL2;
    c.e3[0] = (c.P[0] - c.B[0]) / GL3; c.e3[1] = (c.P[1] - c.B[1]) / GL3;
    c.phi2 = atan2f(c.e2[1], c.e2[0]);
    c.phi3 = atan2f(c.e3[1], c.e3[0]);
    // velocidades: A' + L2·φ2'·e2⊥ = B' + L3·φ3'·e3⊥    (⊥ = giro de +90°)
    //   → L2·e2⊥·φ2' − L3·e3⊥·φ3' = B' − A'
    float a11 = -GL2 * c.e2[1], a12 = GL3 * c.e3[1];
    float a21 = GL2 * c.e2[0], a22 = -GL3 * c.e3[0];
    float det = a11 * a22 - a12 * a21;
    if (fabsf(det) < 1e-9f) { c.ok = false; return false; }
    // columna q1: B' − A' = −L1·e1⊥ ;  columna q2: B' − A' = L4·e4⊥
    float r1x = GL1 * s1, r1y = -GL1 * c1;          // −L1·(−s1, c1)
    float r2x = -GL4 * s2, r2y = GL4 * c2;          //  L4·(−s2, c2)
    c.G[0][0] = (r1x * a22 - a12 * r1y) / det;
    c.G[1][0] = (a11 * r1y - r1x * a21) / det;
    c.G[0][1] = (r2x * a22 - a12 * r2y) / det;
    c.G[1][1] = (a11 * r2y - r2x * a21) / det;
    c.ok = true;
    return true;
}

// Aceleraciones angulares de los distales dadas q', q''.
inline void acelDistales(const Cin &c, const float w[2], const float wd[2], float &p2d, float &p3d,
                         float &f2, float &f3) {
    f2 = c.G[0][0] * w[0] + c.G[0][1] * w[1];
    f3 = c.G[1][0] * w[0] + c.G[1][1] * w[1];
    float c1 = cosf(c.q1), s1 = sinf(c.q1), c2 = cosf(c.q2), s2 = sinf(c.q2);
    // A'' = L1(q1''·e1⊥ − q1'²·e1);  B'' = L4(q2''·e4⊥ − q2'²·e4)
    float Ax = GL1 * (-wd[0] * s1 - w[0] * w[0] * c1), Ay = GL1 * (wd[0] * c1 - w[0] * w[0] * s1);
    float Bx = GL4 * (-wd[1] * s2 - w[1] * w[1] * c2), By = GL4 * (wd[1] * c2 - w[1] * w[1] * s2);
    // A'' + L2(φ2''e2⊥ − φ2'²e2) = B'' + L3(φ3''e3⊥ − φ3'²e3)
    float rx = Bx - Ax - GL3 * f3 * f3 * c.e3[0] + GL2 * f2 * f2 * c.e2[0];
    float ry = By - Ay - GL3 * f3 * f3 * c.e3[1] + GL2 * f2 * f2 * c.e2[1];
    float a11 = -GL2 * c.e2[1], a12 = GL3 * c.e3[1];
    float a21 = GL2 * c.e2[0], a22 = -GL3 * c.e3[0];
    float det = a11 * a22 - a12 * a21;
    p2d = (rx * a22 - a12 * ry) / det;
    p3d = (a11 * ry - rx * a21) / det;
}

// M(q) (2×2) con las masas medidas: rotores, proximales, distales y efector como masa puntual.
inline bool masa(float q1, float q2, float M[2][2]) {
    Cin c;
    if (!cinematica(q1, q2, c)) return false;
    float s1 = sinf(q1), c1 = cosf(q1), s2 = sinf(q2), c2 = cosf(q2);
    float Ip = M_PROX * GL1 * GL1 / 12.0f, Id = M_DIST * GL2 * GL2 / 12.0f;
    float m11 = J_ROTOR + M_PROX * CM_PROX * CM_PROX + Ip;
    float m22 = J_ROTOR + M_PROX * CM_PROX * CM_PROX + M_PROX * GL4 * GL4 / 12.0f;
    float m12 = 0.0f;
    // Jacobianos (2×2 cada uno) de las velocidades lineales
    float JA[2][2] = {{-GL1 * s1, 0}, {GL1 * c1, 0}};
    float JB[2][2] = {{0, -GL4 * s2}, {0, GL4 * c2}};
    float e2p[2] = {-c.e2[1], c.e2[0]}, e3p[2] = {-c.e3[1], c.e3[0]};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            // distal izquierdo: v = A' + rd·φ2'·e2⊥ ;  distal derecho: v = B' + rd·φ3'·e3⊥ ;  efector: v = B' + L3·φ3'·e3⊥
            float vi[3][2], vj[3][2];
            for (int k = 0; k < 2; k++) {
                vi[0][k] = JA[k][i] + CM_DIST * c.G[0][i] * e2p[k];
                vj[0][k] = JA[k][j] + CM_DIST * c.G[0][j] * e2p[k];
                vi[1][k] = JB[k][i] + CM_DIST * c.G[1][i] * e3p[k];
                vj[1][k] = JB[k][j] + CM_DIST * c.G[1][j] * e3p[k];
                vi[2][k] = JB[k][i] + GL3 * c.G[1][i] * e3p[k];
                vj[2][k] = JB[k][j] + GL3 * c.G[1][j] * e3p[k];
            }
            float s = M_DIST * (vi[0][0] * vj[0][0] + vi[0][1] * vj[0][1])
                    + M_DIST * (vi[1][0] * vj[1][0] + vi[1][1] * vj[1][1])
                    + M_EFEC * (vi[2][0] * vj[2][0] + vi[2][1] * vj[2][1])
                    + Id * (c.G[0][i] * c.G[0][j] + c.G[1][i] * c.G[1][j]);
            if (i == 0 && j == 0) m11 += s;
            else if (i == 1 && j == 1) m22 += s;
            else if (i == 0 && j == 1) m12 += s;
        }
    }
    M[0][0] = m11; M[1][1] = m22; M[0][1] = M[1][0] = m12;
    return true;
}

// ---------------------------------------------------------------- algebra 7×7
const int NX = 7;
typedef float Mat[NX][NX];

inline void matMul(const Mat A, const Mat B, Mat C) {
    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NX; j++) {
            float s = 0;
            for (int k = 0; k < NX; k++) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

// e^(A·dt) por escalado y cuadrados (Taylor de orden 4 en dt/2^s)
inline void expmDt(const Mat A, float dt, Mat E) {
    float nrm = 0;
    for (int i = 0; i < NX; i++) {
        float r = 0;
        for (int j = 0; j < NX; j++) r += fabsf(A[i][j]);
        if (r > nrm) nrm = r;
    }
    int s = 0;
    float h = dt;
    while (nrm * h > 0.25f && s < 12) { h *= 0.5f; s++; }
    Mat X, X2, X3, T;
    for (int i = 0; i < NX; i++) for (int j = 0; j < NX; j++) X[i][j] = A[i][j] * h;
    matMul(X, X, X2);
    matMul(X2, X, X3);
    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NX; j++) {
            float x4 = 0;
            for (int k = 0; k < NX; k++) x4 += X3[i][k] * X[k][j];
            E[i][j] = (i == j ? 1.0f : 0.0f) + X[i][j] + X2[i][j] * 0.5f + X3[i][j] / 6.0f + x4 / 24.0f;
        }
    for (int k = 0; k < s; k++) {
        matMul(E, E, T);
        memcpy(E, T, sizeof(Mat));
    }
}

// ---------------------------------------------------------------- EKF
struct Entrada {
    float r[2];          // angulos emitidos en t − tau (rad)
    float rd[2];         // su derivada (rad/s)
};

class EKF {
public:
    Param p;
    float x[NX];
    Mat   P;
    float M[2][2], Mi[2][2], dM1[2][2], dM2[2][2];
    bool  iniciado = false;
    float inov[3] = {0, 0, 0};

    void reiniciar(float q1, float q2) {
        memset(x, 0, sizeof(x));
        x[0] = q1; x[1] = q2;
        memset(P, 0, sizeof(P));
        P[0][0] = P[1][1] = 1e-6f;
        P[2][2] = P[3][3] = 1e-4f;
        P[4][4] = P[5][5] = p.sd_d * p.sd_d;
        P[6][6] = 1e-4f;
        iniciado = true;
    }

    // M, M⁻¹ y ∂M/∂q en la postura de la referencia (como en el EKF de Python, que usa M sobre el comando)
    bool prepararMasa(const Entrada &u) {
        if (!masa(u.r[0], u.r[1], M)) return false;
        float det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        Mi[0][0] = M[1][1] / det; Mi[1][1] = M[0][0] / det;
        Mi[0][1] = -M[0][1] / det; Mi[1][0] = -M[1][0] / det;
        const float e = 2e-3f;
        float Mp[2][2], Mm[2][2];
        if (!masa(u.r[0] + e, u.r[1], Mp) || !masa(u.r[0] - e, u.r[1], Mm)) return false;
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) dM1[i][j] = (Mp[i][j] - Mm[i][j]) / (2 * e);
        if (!masa(u.r[0], u.r[1] + e, Mp) || !masa(u.r[0], u.r[1] - e, Mm)) return false;
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) dM2[i][j] = (Mp[i][j] - Mm[i][j]) / (2 * e);
        return true;
    }

    // aceleraciones articulares del modelo
    void deriv(const float *xx, const Entrada &u, float wd[2]) const {
        const float w0 = xx[2], w1 = xx[3];
        float md[2][2];
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) md[i][j] = dM1[i][j] * w0 + dM2[i][j] * w1;
        float g1 = 0.5f * (w0 * (dM1[0][0] * w0 + dM1[0][1] * w1) + w1 * (dM1[1][0] * w0 + dM1[1][1] * w1));
        float g2 = 0.5f * (w0 * (dM2[0][0] * w0 + dM2[0][1] * w1) + w1 * (dM2[1][0] * w0 + dM2[1][1] * w1));
        float cor1 = md[0][0] * w0 + md[0][1] * w1 - g1;
        float cor2 = md[1][0] * w0 + md[1][1] * w1 - g2;
        float t1 = p.K1 * (u.r[0] - xx[0]) + p.B1 * (p.ff * u.rd[0] - w0) - cor1 + xx[4];
        float t2 = p.K2 * (u.r[1] - xx[1]) + p.B2 * (p.ff * u.rd[1] - w1) - cor2 + xx[5];
        wd[0] = Mi[0][0] * t1 + Mi[0][1] * t2;
        wd[1] = Mi[1][0] * t1 + Mi[1][1] * t2;
    }

    // mediciones predichas [gz, ax, ay]
    bool medir(const float *xx, const Entrada &u, float h[3]) const {
        Cin c;
        if (!cinematica(xx[0], xx[1], c)) return false;
        float wd[2];
        deriv(xx, u, wd);
        float w[2] = {xx[2], xx[3]};
        float p2d, p3d, f2, f3;
        acelDistales(c, w, wd, p2d, p3d, f2, f3);
        h[0] = K_GIRO * f3 + xx[6];
        // chip: s = B + (L3 − R)·e3  →  s'' = B'' + (L3 − R)(φ3''·e3⊥ − φ3'²·e3)
        float c2 = cosf(xx[1]), s2 = sinf(xx[1]);
        float Bx = GL4 * (-wd[1] * s2 - w[1] * w[1] * c2), By = GL4 * (wd[1] * c2 - w[1] * w[1] * s2);
        float l = GL3 - R_IMU;
        float ax = Bx + l * (-p3d * c.e3[1] - f3 * f3 * c.e3[0]);
        float ay = By + l * (p3d * c.e3[0] - f3 * f3 * c.e3[1]);
        float ang = -(c.phi3 + p.psi);
        float ca = cosf(ang), sa = sinf(ang);
        float f = c.phi3;
        h[1] = ca * ax - sa * ay + p.gcx + p.gp1 * cosf(f) + p.gp2 * sinf(f);
        h[2] = sa * ax + ca * ay + p.gcy - p.gp1 * sinf(f) + p.gp2 * cosf(f);
        return true;
    }

    // Un paso: predicción de dt con la entrada anterior y corrección con la medición z (o sin corrección).
    // usarAcc = false: solo el giroscopio.  Devuelve false si algo no es valido.
    bool paso(float dt, const Entrada &uPrev, const Entrada &u, const float z[3], bool medicionValida,
              bool usarAcc = true) {
        if (!iniciado) reiniciar(u.r[0], u.r[1]);
        // ---- prediccion (RK4, 2 subpasos) con el modelo en la postura de la entrada anterior
        if (dt > 0) {
            if (!prepararMasa(uPrev)) return false;
            float h = dt * 0.5f;
            for (int sp = 0; sp < 2; sp++) {
                float k[4][NX], xt[NX];
                for (int st = 0; st < 4; st++) {
                    const float c = (st == 0) ? 0.0f : (st == 3 ? 1.0f : 0.5f);
                    for (int i = 0; i < NX; i++) xt[i] = x[i] + (st == 0 ? 0.0f : c * h * k[st - 1][i]);
                    float wd[2];
                    deriv(xt, uPrev, wd);
                    k[st][0] = xt[2]; k[st][1] = xt[3];
                    k[st][2] = wd[0]; k[st][3] = wd[1];
                    k[st][4] = -xt[4] / p.tau_d; k[st][5] = -xt[5] / p.tau_d;
                    k[st][6] = 0.0f;
                }
                for (int i = 0; i < NX; i++) x[i] += h / 6.0f * (k[0][i] + 2 * k[1][i] + 2 * k[2][i] + k[3][i]);
            }
            Mat A, Phi, T;
            memset(A, 0, sizeof(A));
            A[0][2] = A[1][3] = 1.0f;
            A[2][0] = -Mi[0][0] * p.K1; A[2][1] = -Mi[0][1] * p.K2;
            A[3][0] = -Mi[1][0] * p.K1; A[3][1] = -Mi[1][1] * p.K2;
            A[2][2] = -Mi[0][0] * p.B1; A[2][3] = -Mi[0][1] * p.B2;
            A[3][2] = -Mi[1][0] * p.B1; A[3][3] = -Mi[1][1] * p.B2;
            A[2][4] = Mi[0][0]; A[2][5] = Mi[0][1];
            A[3][4] = Mi[1][0]; A[3][5] = Mi[1][1];
            A[4][4] = A[5][5] = -1.0f / p.tau_d;
            expmDt(A, dt, Phi);
            // P = Phi·P·Phiᵀ + Q
            matMul(Phi, P, T);
            for (int i = 0; i < NX; i++)
                for (int j = 0; j < NX; j++) {
                    float s = 0;
                    for (int k = 0; k < NX; k++) s += T[i][k] * Phi[j][k];
                    P[i][j] = s;
                }
            float qd = 2.0f * p.sd_d * p.sd_d / p.tau_d;
            P[0][0] += p.s_q * p.s_q * dt; P[1][1] += p.s_q * p.s_q * dt;
            P[2][2] += p.s_w * p.s_w * dt; P[3][3] += p.s_w * p.s_w * dt;
            P[4][4] += qd * dt; P[5][5] += qd * dt;
            P[6][6] += p.s_b * p.s_b * dt;
        }
        if (!medicionValida) return true;
        // ---- correccion con la entrada actual
        if (!prepararMasa(u)) return false;
        const int nz = usarAcc ? 3 : 1;
        float hx[3];
        if (!medir(x, u, hx)) return false;
        float H[3][NX];
        for (int j = 0; j < NX; j++) {
            float e = (j < 2) ? 1e-4f : (j < 4 ? 1e-3f : (j < 6 ? 1e-3f : 1e-4f));
            float xp[NX], xm[NX], hp[3], hm[3];
            memcpy(xp, x, sizeof(x)); memcpy(xm, x, sizeof(x));
            xp[j] += e; xm[j] -= e;
            if (!medir(xp, u, hp) || !medir(xm, u, hm)) return false;
            for (int i = 0; i < nz; i++) H[i][j] = (hp[i] - hm[i]) / (2 * e);
        }
        float y[3], S[3][3], PHt[NX][3];
        for (int i = 0; i < nz; i++) y[i] = z[i] - hx[i];
        for (int i = 0; i < NX; i++)
            for (int k = 0; k < nz; k++) {
                float s = 0;
                for (int j = 0; j < NX; j++) s += P[i][j] * H[k][j];
                PHt[i][k] = s;
            }
        for (int a = 0; a < nz; a++)
            for (int b = 0; b < nz; b++) {
                float s = 0;
                for (int j = 0; j < NX; j++) s += H[a][j] * PHt[j][b];
                S[a][b] = s + (a == b ? (a == 0 ? p.r_gz * p.r_gz : p.r_acc * p.r_acc) : 0.0f);
            }
        // inversa de S (1×1 o 3×3)
        float Si[3][3];
        if (nz == 1) Si[0][0] = 1.0f / S[0][0];
        else {
            float d = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
                    - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
                    + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
            if (fabsf(d) < 1e-30f) return false;
            Si[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) / d;
            Si[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) / d;
            Si[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) / d;
            Si[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) / d;
            Si[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / d;
            Si[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) / d;
            Si[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) / d;
            Si[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) / d;
            Si[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) / d;
        }
        float Kg[NX][3];
        for (int i = 0; i < NX; i++)
            for (int b = 0; b < nz; b++) {
                float s = 0;
                for (int a = 0; a < nz; a++) s += PHt[i][a] * Si[a][b];
                Kg[i][b] = s;
            }
        for (int i = 0; i < NX; i++) {
            float s = 0;
            for (int b = 0; b < nz; b++) s += Kg[i][b] * y[b];
            x[i] += s;
        }
        // P = (I − K·H)·P
        Mat T;
        for (int i = 0; i < NX; i++)
            for (int j = 0; j < NX; j++) {
                float s = 0;
                for (int b = 0; b < nz; b++) s += Kg[i][b] * H[b][j];
                T[i][j] = (i == j ? 1.0f : 0.0f) - s;
            }
        Mat Pn;
        matMul(T, P, Pn);
        for (int i = 0; i < NX; i++)
            for (int j = 0; j < NX; j++) P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
        for (int i = 0; i < 3; i++) inov[i] = (i < nz) ? y[i] : 0.0f;
        for (int i = 0; i < NX; i++) if (!(x[i] == x[i])) return false;   // NaN
        return true;
    }
};

// ---------------------------------------------------------------- ley de correccion
class Correccion {
public:
    float delta[2] = {0, 0};                  // ultima correccion (rad), ya recortada
    float hist[lqr::N_RET][2];                // Δ aplicados en las muestras anteriores (k−1 … k−N)
    float lav[2] = {0, 0};                    // estado del filtro de lavado (parte lenta de la ley)

    void reiniciar() { memset(delta, 0, sizeof(delta)); memset(hist, 0, sizeof(hist)); memset(lav, 0, sizeof(lav)); }

    // qPlan/vPlan: comando sin correccion en t − tau; (xp, yp): posicion del plan (m) para la tabla LQR
    void calcular(const Param &p, const EKF &f, const float qPlan[2], const float vPlan[2], float xp, float yp,
                  bool aplicada, float dt = 0.01f) {
        float dl[2];
        if (p.ley == 1) {
            float F[2][lqr::NZ];
            lqr::ganancia(xp, yp, F);
            float z[lqr::NZ];
            z[0] = f.x[0] - qPlan[0]; z[1] = f.x[1] - qPlan[1];
            z[2] = f.x[2] - vPlan[0]; z[3] = f.x[3] - vPlan[1];
            for (int j = 0; j < lqr::N_RET; j++) { z[4 + 2 * j] = hist[j][0]; z[5 + 2 * j] = hist[j][1]; }
            for (int i = 0; i < 2; i++) {
                float s = 0;
                for (int j = 0; j < lqr::NZ; j++) s += F[i][j] * z[j];
                dl[i] = -s;
            }
        } else {
            float K[2] = {p.K1, p.K2};
            float Mii[2] = {f.M[0][0], f.M[1][1]};
            for (int i = 0; i < 2; i++) {
                float e = f.x[2 + i] - vPlan[i];
                float a = fabsf(e) - p.zona;
                e = a > 0 ? (e > 0 ? a : -a) : 0.0f;
                float Dd = 2.0f * p.alfa * sqrtf(K[i] * Mii[i]);
                dl[i] = -(Dd / K[i]) * e;
            }
        }
        // lavado (pasa-altos de primer orden): Δ = ley − (ley filtrada con constante p.lavado)
        if (p.lavado > 0.0f) {
            float g = dt / p.lavado;
            if (g > 1.0f) g = 1.0f;
            for (int i = 0; i < 2; i++) {
                if (dl[i] == dl[i]) lav[i] += g * (dl[i] - lav[i]);
                dl[i] -= lav[i];
            }
        }
        for (int i = 0; i < 2; i++) {
            float lo = delta[i] - p.dpaso, hi = delta[i] + p.dpaso;
            if (dl[i] < lo) dl[i] = lo;
            if (dl[i] > hi) dl[i] = hi;
            if (dl[i] > p.dmax) dl[i] = p.dmax;
            if (dl[i] < -p.dmax) dl[i] = -p.dmax;
            if (!(dl[i] == dl[i])) dl[i] = 0.0f;
            delta[i] = dl[i];
        }
        // historia de lo APLICADO (en modo sombra no se aplica nada)
        for (int j = lqr::N_RET - 1; j > 0; j--) { hist[j][0] = hist[j - 1][0]; hist[j][1] = hist[j - 1][1]; }
        hist[0][0] = aplicada ? delta[0] : 0.0f;
        hist[0][1] = aplicada ? delta[1] : 0.0f;
    }
};

}  // namespace ekf5b
#endif  // EKF_5B_H

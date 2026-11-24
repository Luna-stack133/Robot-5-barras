"""
Diseño LQR de la corrección de la referencia, con el retardo incluido (etapa 4).

Error respecto al comando que sigue el servo:  e_q = q − q_c(t−τ),  e_w = w − q̇_c(t−τ)
Modelo linealizado en la postura q (M(q) del modelo identificado):
    ė_q = e_w
    M·ė_w = −K·e_q − B·e_w + K·u(t − Td)            (u = Δq que se suma a los pulsos)
Td = d_io + τ: la corrección tarda Td en llegar al servo (lectura IMU + planificador + retardo del driver).
Discretizado a 100 Hz con retenedor de orden cero y aumentado con las N = Td/dt entradas pendientes:
    z = [e_q, e_w, u_{k−1}, …, u_{k−N}]   →   LQR discreto (ecuación de Riccati)   →   u_k = −F·z_k
Pesos (regla de Bryson): error de posición tolerable ep [°], de velocidad ev [°/s], corrección tolerable du [°].
"""
import math

import numpy as np
from scipy.linalg import expm, solve_discrete_are

DT = 0.01


def kb(p):
    return (np.diag([p.get("K1", p.get("K")), p.get("K2", p.get("K"))]),
            np.diag([p.get("B1", p.get("B")), p.get("B2", p.get("B"))]))


def riccati_iterativo(A, B, Q, R, it=5000, tol=1e-10):
    P = Q.copy()
    for _ in range(it):
        G = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        Pn = Q + A.T @ P @ (A - B @ G)
        Pn = 0.5 * (Pn + Pn.T)
        if np.abs(Pn - P).max() < tol * max(1.0, np.abs(P).max()):
            return Pn
        P = Pn
    return P


def ganancia(M, p, td=0.043, ep=0.5, ev=5.0, du=1.0, dt=DT):
    K, B = kb(p)
    Mi = np.linalg.inv(M)
    A = np.zeros((4, 4))
    A[0:2, 2:4] = np.eye(2)
    A[2:4, 0:2] = -Mi @ K
    A[2:4, 2:4] = -Mi @ B
    Bu = np.zeros((4, 2))
    Bu[2:4, :] = Mi @ K
    # ZOH exacto
    S = np.zeros((6, 6))
    S[:4, :4] = A
    S[:4, 4:] = Bu
    E = expm(S * dt)
    Ad, Bd = E[:4, :4], E[:4, 4:]
    N = max(1, int(round(td / dt)))
    nz = 4 + 2 * N
    Az = np.zeros((nz, nz))
    Bz = np.zeros((nz, 2))
    Az[:4, :4] = Ad
    Az[:4, 4 + 2 * (N - 1):4 + 2 * N] = Bd          # la entrada que llega ahora es u_{k−N}
    Bz[4:6, :] = np.eye(2)                          # u_{k−1} ← u_k
    for j in range(1, N):
        Az[4 + 2 * j:6 + 2 * j, 4 + 2 * (j - 1):6 + 2 * (j - 1)] = np.eye(2)
    qp = 1 / math.radians(ep) ** 2 if ep > 0 else 0.0
    qv = 1 / math.radians(ev) ** 2
    Q = np.zeros((nz, nz))
    Q[0:2, 0:2] = qp * np.eye(2)
    Q[2:4, 2:4] = qv * np.eye(2)
    R = np.eye(2) / math.radians(du) ** 2
    try:
        P = solve_discrete_are(Az, Bz, Q, R)
    except (ValueError, np.linalg.LinAlgError):
        P = riccati_iterativo(Az, Bz, Q, R)
    F = np.linalg.solve(R + Bz.T @ P @ Bz, Bz.T @ P @ Az)
    cl = np.abs(np.linalg.eigvals(Az - Bz @ F)).max()
    return F, N, cl

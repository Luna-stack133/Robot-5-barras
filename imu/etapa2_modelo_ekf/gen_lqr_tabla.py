"""Genera robot_5bar_esp32_v13/lqr_tabla.h: ganancias LQR (con retardo) en una rejilla de la zona de trabajo."""
import json, math, sys
import numpy as np
sys.path.insert(0, '.')
import lqr5b, modelo5b as m, video_real as vr

p = json.load(open(sys.argv[1]))['parametros']
salida = sys.argv[2]
TD, EP, EV, DU = 0.043, 0.5, 10.0, 1.0
X0, DX, NXG = -0.060, 0.015, 19
Y0, DY, NYG = 0.120, 0.015, 13
F = np.full((NXG, NYG, 2, 12), np.nan)
for i in range(NXG):
    for j in range(NYG):
        x, y = X0 + i * DX, Y0 + j * DY
        q = vr.ik(x, y)
        if q is None or not np.all(np.isfinite(q)):
            continue
        try:
            M, _ = m.masa(*q)
            Fi, N, cl = lqr5b.ganancia(M, p, td=TD, ep=EP, ev=EV, du=DU)
        except Exception:
            continue
        if N != 4 or cl >= 1.0:
            continue
        F[i, j] = Fi
ok = np.isfinite(F[:, :, 0, 0])
print('celdas válidas %d de %d' % (ok.sum(), ok.size))
idx = np.argwhere(ok)
for i in range(NXG):
    for j in range(NYG):
        if not ok[i, j]:
            k = np.argmin(((idx - [i, j]) ** 2).sum(1))
            F[i, j] = F[tuple(idx[k])]
with open(salida, 'w') as f:
    f.write('// Generado por etapa2/gen_lqr_tabla.py — NO editar a mano.\n')
    f.write('// LQR discreto (100 Hz) del error respecto al comando, con %d muestras de retardo (Td = %.0f ms),\n' % (4, TD * 1000))
    f.write('// pesos de Bryson: e_q %.2f°, e_w %.1f°/s, Δq %.1f°.  Modelo: K1 %.4f K2 %.4f B1 %.5f B2 %.5f ff %.3f tau %.4f\n'
            % (EP, EV, DU, p['K1'], p['K2'], p['B1'], p['B2'], p['ff'], p['tau']))
    f.write('// Rejilla: x = %.3f + i·%.3f (i < %d), y = %.3f + j·%.3f (j < %d) m; %d de %d celdas alcanzables\n'
            % (X0, DX, NXG, Y0, DY, NYG, ok.sum(), ok.size))
    f.write('#ifndef LQR_TABLA_H\n#define LQR_TABLA_H\nnamespace lqr {\n')
    f.write('const int N_RET = 4;\nconst int NZ = 12;\n')
    f.write('const float X0 = %.4ff, DX = %.4ff, Y0 = %.4ff, DY = %.4ff;\nconst int NXG = %d, NYG = %d;\n' % (X0, DX, Y0, DY, NXG, NYG))
    f.write('const float TABLA[%d][%d][2][12] = {\n' % (NXG, NYG))
    for i in range(NXG):
        f.write(' {\n')
        for j in range(NYG):
            f.write('  {' + ', '.join('{' + ', '.join('%.5ef' % v for v in F[i, j, r]) + '}' for r in range(2)) + '},\n')
        f.write(' },\n')
    f.write('};\n')
    f.write('''// interpolacion bilineal de la ganancia en la posicion del plan (x, y en m)
inline void ganancia(float x, float y, float F[2][NZ]) {
    float fi = (x - X0) / DX, fj = (y - Y0) / DY;
    if (fi < 0) fi = 0;
    if (fi > NXG - 1.001f) fi = NXG - 1.001f;
    if (fj < 0) fj = 0;
    if (fj > NYG - 1.001f) fj = NYG - 1.001f;
    int i = (int)fi, j = (int)fj;
    float a = fi - i, b = fj - j;
    for (int r = 0; r < 2; r++)
        for (int k = 0; k < NZ; k++)
            F[r][k] = (1 - a) * (1 - b) * TABLA[i][j][r][k] + a * (1 - b) * TABLA[i + 1][j][r][k]
                    + (1 - a) * b * TABLA[i][j + 1][r][k] + a * b * TABLA[i + 1][j + 1][r][k];
}
}  // namespace lqr
#endif
''')
print('escrito', salida)

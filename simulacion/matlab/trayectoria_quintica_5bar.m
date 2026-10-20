function traj = trayectoria_quintica_5bar(P_pick, P_place, p, fs, Vmax, Amax, cfgInv)
% ═════════════════════════════════════════════════════════════════════════
% GENERADOR DE TRAYECTORIA QUÍNTICA PARA ROBOT PARALELO 5-BARRAS
%
% ► Genera trayectoria cartesiana tipo pick-and-place
% ► Perfil quíntico:
%
%    q(0)=q0,   q(T)=qf
%    q̇(0)=0,   q̇(T)=0
%    q̈(0)=0,   q̈(T)=0
%
% ► Polinomio:
%
%    s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
%
% ► Incluye:
%    - Posición cartesiana
%    - Velocidad cartesiana
%    - Aceleración cartesiana
%    - Cinemática inversa
%    - Velocidad articular mediante Jacobiano inverso
%    - Aceleración articular numérica
%    - Índice de manipulabilidad
%
% ENTRADAS:
% -----------------------------------------------------------------------
% P_pick   : [x y] punto inicial
% P_place  : [x y] punto final
% p        : estructura de parámetros del robot
% fs       : frecuencia de muestreo [Hz]
% Vmax     : velocidad máxima cartesiana [m/s]
% Amax     : aceleración máxima cartesiana [m/s²]
% cfgInv   : configuración del Jacobiano inverso
%
% SALIDA:
% -----------------------------------------------------------------------
% traj     : estructura con toda la trayectoria
%
% DEPENDENCIAS:
% -----------------------------------------------------------------------
% ik5bar.m
% jacobiano_analitico.m
% inverse_jacobian.m
%
% ═════════════════════════════════════════════════════════════════════════

fprintf('\n══ GENERACIÓN DE TRAYECTORIA QUÍNTICA ══\n')

%% ───────────────────────────────────────────────────────────────────────
% VERIFICACIÓN DE ALCANZABILIDAD
% ────────────────────────────────────────────────────────────────────────

[th1_pick, th2_pick, ok1] = ik5bar(P_pick(1),  P_pick(2),  p, 3);
[th1_place,th2_place,ok2] = ik5bar(P_place(1), P_place(2), p, 3);

fprintf('\nPick  P=(%.3f, %.3f): %s  θ=[%.2f°, %.2f°]\n',...
    P_pick(1),P_pick(2),ternario(ok1,'✓','✗'),...
    rad2deg(th1_pick),rad2deg(th2_pick))

fprintf('Place P=(%.3f, %.3f): %s  θ=[%.2f°, %.2f°]\n',...
    P_place(1),P_place(2),ternario(ok2,'✓','✗'),...
    rad2deg(th1_place),rad2deg(th2_place))

if ~(ok1 && ok2)
    error('Uno o ambos puntos no son alcanzables.')
end

%% ───────────────────────────────────────────────────────────────────────
% DISTANCIA TOTAL
% ────────────────────────────────────────────────────────────────────────

D = norm(P_place - P_pick);

fprintf('\nDistancia cartesiana: %.4f m\n', D)

%% ───────────────────────────────────────────────────────────────────────
% TIEMPO MÍNIMO SEGÚN RESTRICCIONES
% ────────────────────────────────────────────────────────────────────────

T_vel = (1.875 * D) / Vmax;
T_acc = sqrt((5.77 * D) / Amax);

T_tray = max(T_vel, T_acc);

fprintf('Tiempo mínimo por velocidad   : %.3f s\n', T_vel)
fprintf('Tiempo mínimo por aceleración : %.3f s\n', T_acc)
fprintf('Tiempo final seleccionado     : %.3f s\n', T_tray)

%% ───────────────────────────────────────────────────────────────────────
% VECTOR TEMPORAL
% ────────────────────────────────────────────────────────────────────────

Ts    = 1/fs;
t_vec = 0:Ts:T_tray;
N     = length(t_vec);

%% ───────────────────────────────────────────────────────────────────────
% POLINOMIO QUÍNTICO
% ────────────────────────────────────────────────────────────────────────

tau = t_vec / T_tray;

s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

ds  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T_tray;

dds = (60*tau - 180*tau.^2 + 120*tau.^3)/(T_tray^2);

%% ───────────────────────────────────────────────────────────────────────
% TRAYECTORIA CARTESIANA
% ────────────────────────────────────────────────────────────────────────

traj.t  = t_vec;

traj.px = P_pick(1) + (P_place(1)-P_pick(1))*s;
traj.py = P_pick(2) + (P_place(2)-P_pick(2))*s;

traj.vx = (P_place(1)-P_pick(1))*ds;
traj.vy = (P_place(2)-P_pick(2))*ds;

traj.ax = (P_place(1)-P_pick(1))*dds;
traj.ay = (P_place(2)-P_pick(2))*dds;

%% ───────────────────────────────────────────────────────────────────────
% PREALOCACIÓN ARTICULAR
% ────────────────────────────────────────────────────────────────────────

traj.q1   = zeros(1,N);
traj.q2   = zeros(1,N);

traj.dq1  = zeros(1,N);
traj.dq2  = zeros(1,N);

traj.ddq1 = zeros(1,N);
traj.ddq2 = zeros(1,N);

traj.manip = zeros(1,N);

%% ───────────────────────────────────────────────────────────────────────
% CINEMÁTICA INVERSA + JACOBIANO
% ────────────────────────────────────────────────────────────────────────

for k = 1:N

    % ── Cinemática inversa ─────────────────────────────────────────────
    [t1k,t2k,ok] = ik5bar(traj.px(k), traj.py(k), p, 3);

    t1k = mod(t1k, 2*pi);
    t2k = mod(t2k, 2*pi);

    if ok

        traj.q1(k) = t1k;
        traj.q2(k) = t2k;

    elseif k > 1

        traj.q1(k) = traj.q1(k-1);
        traj.q2(k) = traj.q2(k-1);

    end

    % ── Jacobiano ──────────────────────────────────────────────────────
    [~,~,Jk] = jacobiano_analitico(traj.q1(k), traj.q2(k), p);

    % ── Manipulabilidad ────────────────────────────────────────────────
    w = sqrt(det(Jk*Jk'));

    traj.manip(k) = w;

    % ── Escalamiento adaptativo cerca de singularidad ─────────────────
    scale = 1.0;

    if w < cfgInv.w_min
        scale = max(0.15, w/cfgInv.w_min);
    end

    v_cart = scale * [traj.vx(k); traj.vy(k)];

    % ── Jacobiano inverso robusto ─────────────────────────────────────
    qd = inverse_jacobian(...
            Jk,...
            v_cart,...
            cfgInv.method,...
            cfgInv.lambda);

    traj.dq1(k) = qd(1);
    traj.dq2(k) = qd(2);

end

%% ───────────────────────────────────────────────────────────────────────
% ACELERACIÓN ARTICULAR
% ────────────────────────────────────────────────────────────────────────

traj.ddq1 = gradient(traj.dq1, Ts);
traj.ddq2 = gradient(traj.dq2, Ts);

%% ───────────────────────────────────────────────────────────────────────
% INFORMACIÓN ADICIONAL
% ────────────────────────────────────────────────────────────────────────

traj.Ts     = Ts;
traj.fs     = fs;
traj.T_tray = T_tray;
traj.D      = D;
traj.Vmax   = Vmax;
traj.Amax   = Amax;

fprintf('\nTrayectoria generada correctamente.\n')

end
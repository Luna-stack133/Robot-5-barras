function traj = trayectoria_multi_5bar( ...
    P,...
    p,...
    fs,...
    Vmax,...
    Amax,...
    cfgInv,...
    closedLoop)

% ══════════════════════════════════════════════════════════════════════════
% GENERADOR DE TRAYECTORIA MULTI-PUNTO
%
% P:
%   matriz Nx2 de puntos cartesianos
%
% closedLoop:
%   true  → vuelve al punto inicial
%   false → trayectoria abierta
%
% ══════════════════════════════════════════════════════════════════════════

if nargin < 7

    closedLoop = false;

end

% ── CERRAR TRAYECTORIA ────────────────────────────────────────────────

if closedLoop

    P = [P; P(1,:)];

end

% ── NÚMERO DE SEGMENTOS ───────────────────────────────────────────────

Nseg = size(P,1) - 1;

% ── INICIALIZAR ───────────────────────────────────────────────────────

traj.t   = [];
traj.px  = [];
traj.py  = [];
traj.q1  = [];
traj.q2  = [];
traj.dq1 = [];
traj.dq2 = [];

t_offset = 0;

% ══════════════════════════════════════════════════════════════════════════
% LOOP SEGMENTOS
% ══════════════════════════════════════════════════════════════════════════

for k = 1:Nseg

    Pi = P(k,:);
    Pf = P(k+1,:);

    traj_k = trayectoria_quintica_5bar( ...
        Pi,...
        Pf,...
        p,...
        fs,...
        Vmax,...
        Amax,...
        cfgInv);
    traj_k.t   = traj_k.t(:);
    traj_k.px  = traj_k.px(:);
    traj_k.py  = traj_k.py(:);
    traj_k.q1  = traj_k.q1(:);
    traj_k.q2  = traj_k.q2(:);
    traj_k.dq1 = traj_k.dq1(:);
    traj_k.dq2 = traj_k.dq2(:);
    % ── EVITAR REPETICIÓN DE PUNTOS ──────────────────────────────────

    if k == 1

        idx = 1:length(traj_k.t);

    else

        idx = 2:length(traj_k.t);

    end

    % ── CONCATENAR ────────────────────────────────────────────────────

    traj.t = [traj.t; traj_k.t(idx) + t_offset];

    traj.px = [

        traj.px;

        traj_k.px(idx)

    ];

    traj.py = [

        traj.py;

        traj_k.py(idx)

    ];

    traj.q1 = [

        traj.q1;

        traj_k.q1(idx)

    ];

    traj.q2 = [

        traj.q2;

        traj_k.q2(idx)

    ];

    traj.dq1 = [

        traj.dq1;

        traj_k.dq1(idx)

    ];

    traj.dq2 = [

        traj.dq2;

        traj_k.dq2(idx)

    ];

    % ── ACTUALIZAR TIEMPO ─────────────────────────────────────────────

    t_offset = traj.t(end);

end

% ── TIEMPO TOTAL ──────────────────────────────────────────────────────

traj.T_tray = traj.t(end);

end
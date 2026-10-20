function animate_trajectory_5bar(traj,p,varargin)
% ════════════════════════════════════════════════════════════════════════
%  ANIMACIÓN + PLOTS DINÁMICOS — ROBOT PARALELO 5 BARRAS
%
%   1) Animación del robot
%   2) theta1 vs tiempo
%   3) theta2 vs tiempo
%   4) Magnitud velocidad efector final
%   5) Magnitud aceleración efector final
%
% ════════════════════════════════════════════════════════════════════════

%% ── OPCIONES ─────────────────────────────────────────────────────────

cfg.dt_anim   = 0.05;
cfg.showTrail = true;
cfg.showText  = true;
cfg.pauseTime = 0.000;

if ~isempty(varargin)

    for k=1:2:length(varargin)

        cfg.(varargin{k}) = varargin{k+1};

    end
end

%% ── VALIDACIONES ─────────────────────────────────────────────────────

requiredFields = {'t','q1','q2','px','py'};

for k=1:length(requiredFields)

    if ~isfield(traj,requiredFields{k})

        error(['Falta traj.' requiredFields{k}])

    end
end

%% ── TIEMPO DE ANIMACIÓN ──────────────────────────────────────────────

t_anim = traj.t(1):cfg.dt_anim:traj.t(end);

idx_anim = zeros(size(t_anim));

for n=1:length(t_anim)

    [~,idx_anim(n)] = min(abs(traj.t - t_anim(n)));

end

%% ── VELOCIDADES Y ACELERACIONES ──────────────────────────────────────

dt = mean(diff(traj.t));

vx = gradient(traj.px,dt);
vy = gradient(traj.py,dt);

v_mag = sqrt(vx.^2 + vy.^2);

ax = gradient(vx,dt);
ay = gradient(vy,dt);

a_mag = sqrt(ax.^2 + ay.^2);

%% ── FIGURA ───────────────────────────────────────────────────────────

figure( ...
    'Name','Animación Robot 5 Barras', ...
    'Position',[50 50 1500 900])

%% ── LÍMITES ──────────────────────────────────────────────────────────

Lmax = p.L1 + p.L2 + 0.1;

xmin = -Lmax;
xmax = p.d + Lmax;

ymin = -Lmax;
ymax =  Lmax;

%% ── BASES ────────────────────────────────────────────────────────────

O1 = [0 0];
O2 = [p.d 0];

%% ── COLORES FIJOS (evita cambios raros de MATLAB) ───────────────────

robotColor = [0 0.4470 0.7410];
trailColor = [0.5 0.5 0.5];

%% ── LOOP DE ANIMACIÓN ────────────────────────────────────────────────

for kk = 1:length(idx_anim)

    k = idx_anim(kk);

    %% ── ESTADOS ACTUALES ────────────────────────────────────────────

    th1 = traj.q1(k);
    th2 = traj.q2(k);

    Px = traj.px(k);
    Py = traj.py(k);

    P = [Px Py];

    %% ── CINEMÁTICA ──────────────────────────────────────────────────

    Ax = p.L1*cos(th1);
    Ay = p.L1*sin(th1);

    Bx = p.d + p.L4*cos(th2);
    By = p.L4*sin(th2);

    A = [Ax Ay];
    B = [Bx By];

    %% ================================================================
    %% ANIMACIÓN ROBOT (MUCHO MÁS GRANDE)
    %% ================================================================

    subplot(3,4,[1 2 5 6 9 10])

    cla
    hold on

    %% ── TRAYECTORIA ────────────────────────────────────────────────

    if cfg.showTrail

        plot(traj.px, traj.py, '--', ...
            'Color',trailColor, ...
            'LineWidth',1.2)

        plot(traj.px(1),traj.py(1), ...
            'go','MarkerSize',7,'LineWidth',1.5)

        plot(traj.px(end),traj.py(end), ...
            'ro','MarkerSize',7,'LineWidth',1.5)

    end

    %% ── ESLABONES ──────────────────────────────────────────────────

    plot([O1(1) A(1)], [O1(2) A(2)], ...
        'Color',robotColor, ...
        'LineWidth',4)

    plot([O2(1) B(1)], [O2(2) B(2)], ...
        'Color',robotColor, ...
        'LineWidth',4)

    plot([A(1) P(1)], [A(2) P(2)], ...
        'Color',robotColor, ...
        'LineWidth',4)

    plot([B(1) P(1)], [B(2) P(2)], ...
        'Color',robotColor, ...
        'LineWidth',4)

    %% ── ARTICULACIONES ─────────────────────────────────────────────

    plot(O1(1),O1(2), ...
        'ko','MarkerSize',8,'LineWidth',2)

    plot(O2(1),O2(2), ...
        'ko','MarkerSize',8,'LineWidth',2)

    plot(A(1),A(2), ...
        'ko','MarkerSize',6,'LineWidth',1.5)

    plot(B(1),B(2), ...
        'ko','MarkerSize',6,'LineWidth',1.5)

    %% ── EFECTOR FINAL ──────────────────────────────────────────────

    plot(P(1),P(2), ...
        'ro', ...
        'MarkerSize',5, ...
        'LineWidth',1.5)

    %% ── TEXTO ──────────────────────────────────────────────────────

    if cfg.showText

        txt = sprintf([ ...
            't = %.2f s\n' ...
            '\\theta_1 = %.1f°\n' ...
            '\\theta_2 = %.1f°'], ...
            traj.t(k), ...
            rad2deg(th1), ...
            rad2deg(th2));

        text(xmin+0.05,ymax-0.05,txt, ...
            'FontSize',11, ...
            'VerticalAlignment','top', ...
            'BackgroundColor','w')

    end

    axis equal

    xlim([xmin xmax])
    ylim([ymin ymax])

    grid on

    xlabel('X [m]')
    ylabel('Y [m]')

    title('Robot Paralelo 5 Barras')

    %% ================================================================
    %% THETA1
    %% ================================================================

    subplot(3,4,3)

    plot(traj.t,rad2deg(traj.q1), ...
        'Color',robotColor, ...
        'LineWidth',1.5)

    hold on

    plot(traj.t(k),rad2deg(traj.q1(k)), ...
        'ro','MarkerSize',4,'LineWidth',1.5)

    grid on

    xlabel('t [s]')
    ylabel('\theta_1 [deg]')

    title('\theta_1')

    %% ================================================================
    %% THETA2
    %% ================================================================

    subplot(3,4,4)

    plot(traj.t,rad2deg(traj.q2), ...
        'Color',robotColor, ...
        'LineWidth',1.5)

    hold on

    plot(traj.t(k),rad2deg(traj.q2(k)), ...
        'ro','MarkerSize',4,'LineWidth',1.5)

    grid on

    xlabel('t [s]')
    ylabel('\theta_2 [deg]')

    title('\theta_2')

    %% ================================================================
    %% VELOCIDAD
    %% ================================================================

    subplot(3,4,7)

    plot(traj.t,v_mag, ...
        'Color',robotColor, ...
        'LineWidth',1.5)

    hold on

    plot(traj.t(k),v_mag(k), ...
        'ro','MarkerSize',4,'LineWidth',1.5)

    grid on

    xlabel('t [s]')
    ylabel('|V| [m/s]')

    title('Velocidad')

    %% ================================================================
    %% ACELERACIÓN
    %% ================================================================

    subplot(3,4,8)

    plot(traj.t,a_mag, ...
        'Color',robotColor, ...
        'LineWidth',1.5)

    hold on

    plot(traj.t(k),a_mag(k), ...
        'ro','MarkerSize',4,'LineWidth',1.5)

    grid on

    xlabel('t [s]')
    ylabel('|a| [m/s²]')

    title('Aceleración')

    %% ── ACTUALIZAR ──────────────────────────────────────────────────

    drawnow

    pause(cfg.pauseTime)

end

end
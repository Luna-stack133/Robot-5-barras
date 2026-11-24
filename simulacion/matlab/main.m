%% ══════════════════════════════════════════════════════════════════════════
%  ROBOT PARALELO 5 BARRAS
%  Simulador de trayectoria
%
%  ► Trayectoria quintic 
%  
%  
% ══════════════════════════════════════════════════════════════════════════
clear; clc; close all

%  SECCIÓN 1── PARÁMETROS DEL ROBOT ───────────────────────────────────────
p.L1 = 0.12;  % brazo izquierdo proximal (geometría v3)
p.L2 = 0.18;  % brazo izquierdo distal
p.L3 = 0.18;  % brazo derecho distal
p.L4 = 0.12;  % brazo derecho proximal
p.d  = 0.15;  % separación entre ejes de los servos
O1x = 0; O1y = 0;  % posicion x,y de punto de barra izquierdo
O2x = p.d; O2y = 0; % posicion x,y de punto de barra derecho
% ── MASAS ───────────────────────────────────────────────────────────────
% masas medidas en balanza (nov. 2026)
p.m1 = 0.115; % proximal izquierdo [kg]
p.m2 = 0.170; % distal izquierdo [kg]
p.m3 = 0.170; % distal derecho [kg]
p.m4 = 0.115; % proximal derecho [kg]
p.m_ef = 0.334;    % efector con torre Z y garra [kg]
% ── CENTROS DE MASA ────────────────────────────────────────────────────
p.lc1 = p.L1/2;
p.lc2 = p.L2/2;
p.lc3 = p.L3/2;
p.lc4 = p.L4/2;
% ── INERCIAS ────────────────────────────────────────────────────────────
p.I1 = (1/12)*p.m1*p.L1^2;
p.I2 = (1/12)*p.m2*p.L2^2;
p.I3 = (1/12)*p.m3*p.L3^2;
p.I4 = (1/12)*p.m4*p.L4^2;
% ── FRICCIÓN ────────────────────────────────────────────────────────────
p.b1 = 0.008;
p.b2 = 0.008;
% ── PARÁMETROS BLDC ─────────────────────────────────────────────────────
motor.Kt   = 0.08;      % [N·m/A]
motor.Ke   = 0.08;      % [V·s/rad]
motor.R    = 0.50;      % [Ohm]
motor.L    = 0.0015;    % [H]   ← NUEVO
motor.bm   = 0.001;     % [N·m·s/rad]
motor.Vbus = 12.0;      % [V]

fprintf('\n══ SECCIÓN 3: Trayectoria Quintica ══\n\n')

% Puntos de vértices de triángulo en zona superior de espacio de trabajo
Puntos = [0.025 0.24
          0.125 0.24
          0.075 0.16];   % [m] alcanzable y lejos de singularidades con la geometría v3

% ── LÍMITES INDUSTRIALES DE MOVIMIENTO ────────────────────────────────
Vmax = 0.2;  % velocidad máxima del efector final en m/s
Amax = 0.25;   % aceleración máxima del efector final en m/s^2
% ── Muestreo de trayectoria ────────────────────────────────
fs      = 100;     % Frecuencia de muestreo 100Hz
% ── CONFIGURACIÓN DEL JACOBIANO INVERSO ───────────────────────────────
cfgInv.method = 'damped'; % Opciones: 'inverse'  'pinv'  'damped'  'damped_adaptive'
cfgInv.lambda = 0.02;
cfgInv.w_min  = 0.02;

traj = trayectoria_multi_5bar(Puntos,p,fs,Vmax,Amax,cfgInv,true);

%% ═════════════════════════════════════════════════════
% ANIMACIÓN + GRÁFICAS
%% ═════════════════════════════════════════════════════

animate_trajectory_5bar(traj,p)




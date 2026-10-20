function [x, y, valid] = forward_kinematics_5bar(theta1, theta2, params, config)
% =========================================================================
% CINEMÁTICA DIRECTA — Robot Paralelo Planar de 5 Barras
% Método: Intersección analítica de dos circunferencias
% =========================================================================
% ENTRADAS:
%   theta1   : ángulo motor izquierdo [rad]
%   theta2   : ángulo motor derecho [rad]
%   params   : estructura de parámetros del robot
%   config   : 1 = solución "superior" (+h), 2 = "inferior" (-h)
%              (por defecto: 2, solución inferior)
%
% SALIDAS:
%   x, y     : posición del efector final [m]
%   valid    : true si la configuración es válida
% =========================================================================

    if nargin < 4
        config = 2;  % 1 solucion superior,  2 Solución inferior (Y negativa, más común)
    end

    % Extraer parámetros
    L1 = params.L1;
    L2 = params.L2;
    L3 = params.L3;
    L4 = params.L4;
    d  = params.d;

    % Inicializar salidas
    x = NaN; y = NaN; valid = false;

    % =====================================================================
    % Paso 1: Calcular posición de articulaciones pasivas A y B
    % Estas son directas porque theta1 y theta2 son conocidos
    % =====================================================================
    Ax = L1 * cos(theta1);
    Ay = L1 * sin(theta1);
    Bx = d + L4 * cos(theta2);
    By = L4 * sin(theta2);

    % =====================================================================
    % Paso 2: Calcular distancia entre A y B
    % =====================================================================
    dAB = sqrt((Bx - Ax)^2 + (By - Ay)^2);

    % =====================================================================
    % Paso 3: Verificar condiciones de intersección
    % =====================================================================
    tol = 1e-9;

    if dAB > (L2 + L3) + tol
        % Los círculos no se intersectan (muy separados)
        %warning('FK: Configuración inválida — A y B demasiado separados (%.4f m > %.4f m)', ...
        %        dAB, L2+L3);
        return;
    end

    if dAB < abs(L2 - L3) - tol
        % Un círculo está dentro del otro
        %warning('FK: Configuración inválida — A y B demasiado cercanos.');
        return;
    end

    if dAB < tol
        % A y B coinciden: singularidad degenerada
        %warning('FK: Singularidad — A y B coinciden.');
        return;
    end

    % =====================================================================
    % Paso 4: Intersección analítica de dos circunferencias
    % Círculo 1: centro A, radio L2
    % Círculo 2: centro B, radio L3
    % =====================================================================

    % 'a' es la distancia desde A hasta el punto medio M sobre la línea AB
    % Se obtiene de: L2² = a² + h²  y  L3² = (dAB-a)² + h²
    a_coef = (L2^2 - L3^2 + dAB^2) / (2 * dAB);

    % 'h' es la distancia perpendicular desde la línea AB hasta los puntos P
    h_sq = L2^2 - a_coef^2;

    if h_sq < -tol
        % Debería ser imposible si pasamos los chequeos anteriores,
        % pero protegemos contra errores numéricos
        warning('FK: Error numérico — h² < 0.');
        return;
    end
    h_coef = sqrt(max(0, h_sq));  % max(0,...) protege contra redondeo

    % Punto medio M sobre la línea AB (a distancia a_coef desde A)
    Mx = Ax + a_coef * (Bx - Ax) / dAB;
    My = Ay + a_coef * (By - Ay) / dAB;

    % Vector perpendicular unitario a la línea AB
    % Si AB apunta en dirección (Bx-Ax, By-Ay)/dAB,
    % la perpendicular es (-(By-Ay), Bx-Ax)/dAB
    perp_x = -(By - Ay) / dAB;
    perp_y =  (Bx - Ax) / dAB;

    % Dos soluciones: P = M ± h * perp
    if config == 1
        % Solución "superior" (signo positivo → P queda a la izquierda de A→B)
        x = Mx + h_coef * perp_x;  % -y
        y = My + h_coef * perp_y;  % -x
    else
        % Solución "inferior" (signo negativo → más usada en práctica)
        x = Mx - h_coef * perp_x;
        y = My - h_coef * perp_y;
    end

    valid = true;

    % =====================================================================
    % Visualización del robot
    % =====================================================================
    if nargout == 0  % Solo grafica si no se piden salidas (modo demo)
        plot_robot_5bar(theta1, theta2, params, x, y);
    end

end


function plot_robot_5bar(theta1, theta2, params, x, y)
% Grafica el robot dado un estado cinemático

    L1 = params.L1; L4 = params.L4; d = params.d;

    % Calcular posiciones
    O1 = [0, 0];
    O2 = [d, 0];
    A  = [L1*cos(theta1), L1*sin(theta1)];
    B  = [d + L4*cos(theta2), L4*sin(theta2)];
    P  = [x, y];

    figure('Name','Cinemática Directa — Robot 5 Barras');
    hold on; grid on; axis equal;

    % Dibujar base
    plot([O1(1), O2(1)], [O1(2), O2(2)], 'k-', 'LineWidth', 4);

    % Cadena izquierda: O1 → A → P
    plot([O1(1), A(1), P(1)], [O1(2), A(2), P(2)], 'b-o', ...
         'LineWidth', 2.5, 'MarkerSize', 10, 'MarkerFaceColor', 'b');

    % Cadena derecha: O2 → B → P
    plot([O2(1), B(1), P(1)], [O2(2), B(2), P(2)], 'r-o', ...
         'LineWidth', 2.5, 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Puntos especiales
    plot(O1(1),O1(2),'ks','MarkerSize',15,'MarkerFaceColor',[0.2 0.2 0.2]);
    plot(O2(1),O2(2),'ks','MarkerSize',15,'MarkerFaceColor',[0.2 0.2 0.2]);
    plot(P(1), P(2), 'g*', 'MarkerSize', 16, 'LineWidth', 2.5);

    % Etiquetas
    text(O1(1)-0.02, O1(2)+0.03, 'O_1 (Motor 1)', 'FontSize', 10);
    text(O2(1)-0.02, O2(2)+0.03, 'O_2 (Motor 2)', 'FontSize', 10);
    text(A(1)+0.01,  A(2)+0.01,  'A', 'FontSize', 11, 'Color', 'b');
    text(B(1)+0.01,  B(2)+0.01,  'B', 'FontSize', 11, 'Color', 'r');
    text(P(1)+0.01,  P(2)-0.02,  'P (Efector)', 'FontSize', 11, 'Color', [0 0.6 0]);

    title(sprintf('FK: θ₁=%.1f°, θ₂=%.1f° → P=(%.3f, %.3f) m', ...
          rad2deg(theta1), rad2deg(theta2), x, y), 'FontSize', 12);
    xlabel('X [m]'); ylabel('Y [m]');
    legend({'Base','Cadena izq.','Cadena der.','Motor O1','Motor O2','Efector P'}, ...
           'Location','best');

    xlim([-0.15, params.d+0.15]);
    ylim([-0.7, 0.4]);
end
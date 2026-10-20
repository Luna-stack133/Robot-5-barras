function qdot = inverse_jacobian(J,v,method,lambda)
% ═════════════════════════════════════════════════════════════════════════
% JACOBIANO INVERSO ROBUSTO
%
% ► Calcula velocidades articulares:
%
%       q̇ = J⁻¹ v
%
% ► Métodos soportados:
%
%   'inverse'          → inversa directa
%   'pinv'             → pseudoinversa de Moore-Penrose
%   'damped'           → Damped Least Squares (DLS)
%   'damped_adaptive'  → DLS adaptativo
%
% ENTRADAS:
% -----------------------------------------------------------------------
% J       : Jacobiano
% v       : velocidad cartesiana deseada
% method  : método de inversión
% lambda  : factor damping
%
% SALIDA:
% -----------------------------------------------------------------------
% qd      : velocidades articulares
%
% ═════════════════════════════════════════════════════════════════════════
    switch lower(method)

        % ============================================================
        % Inversa clásica
        % ============================================================

        case 'inverse'

            qdot = J \ v;

        % ============================================================
        % Pseudoinversa Moore-Penrose
        % ============================================================

        case 'pinv'

            qdot = pinv(J) * v;

        % ============================================================
        % Damped Least Squares
        % ============================================================

        case 'damped'

            qdot = J' * ((J*J' + lambda^2*eye(2)) \ v);

        % ============================================================
        % Damping adaptativo
        % ============================================================

        case 'damped_adaptive'

            w = sqrt(det(J*J'));

            lambda_eff = lambda;

            if w < 0.05

                lambda_eff = lambda*(0.05/w);

            end

            qdot = J' * ((J*J' + lambda_eff^2*eye(2)) \ v);

        otherwise

            error('Método Jacobiano no válido');

    end

end
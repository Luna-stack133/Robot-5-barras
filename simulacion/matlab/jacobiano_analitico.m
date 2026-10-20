%% ── Jacobiano Analítico ──────────────────────────────────────────────────
function [J_P,J_th,J,alpha1,alpha2] = jacobiano_analitico(th1,th2,p)
    [xP,yP,ok]=forward_kinematics_5bar(th1,th2,p);
    if ~ok
        J_P=eye(2); J_th=eye(2); J=eye(2); alpha1=0; alpha2=0; return
    end
    Ax=p.L1*cos(th1); Ay=p.L1*sin(th1);
    Bx=p.d+p.L4*cos(th2); By=p.L4*sin(th2);
    alpha1 = atan2(yP-Ay, xP-Ax);
    alpha2 = atan2(yP-By, xP-Bx);
    % J_P = [cos α₁  sin α₁; cos α₂  sin α₂]
    J_P = [cos(alpha1) sin(alpha1); cos(alpha2) sin(alpha2)];
    % J_θ = diag(L₁·sin(α₁-θ₁), L₄·sin(α₂-θ₂))
    J_th = diag([p.L1*sin(alpha1-th1), p.L4*sin(alpha2-th2)]);
    % J = J_P⁻¹ · J_θ
    if abs(det(J_P)) < 1e-10
        J = zeros(2,2);
    else
        J = J_P \ J_th;
    end
end
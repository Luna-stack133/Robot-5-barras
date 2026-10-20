%% ── Cinemática Inversa ───────────────────────────────────────────────────
function [th1,th2,valid] = ik5bar(x,y,p,config)
    if nargin<4, config=3; end
    th1=NaN; th2=NaN; valid=false;
    r1 = hypot(x,y);
    if r1>p.L1+p.L2+1e-9 || r1<abs(p.L1-p.L2)-1e-9, return; end
    phi1 = atan2(y,x);
    b1   = acos(max(-1,min(1,(r1^2+p.L1^2-p.L2^2)/(2*r1*p.L1))));
    r2   = hypot(x-p.d,y);
    if r2>p.L4+p.L3+1e-9 || r2<abs(p.L4-p.L3)-1e-9, return; end
    phi2 = atan2(y,x-p.d);
    b2   = acos(max(-1,min(1,(r2^2+p.L4^2-p.L3^2)/(2*r2*p.L4))));
    sgn  = [1 -1;-1 1;1 1;-1 -1];
    th1  = phi1+sgn(config,1)*b1;
    th2  = phi2+sgn(config,2)*b2;
    valid= true;
end
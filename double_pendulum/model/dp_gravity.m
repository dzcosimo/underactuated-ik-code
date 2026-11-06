function G = dp_gravity(q, p)
%DP_GRAVITY  Potential force G(q) = dU/dq of Eq. (6): gravity on the tip masses plus the
%   parallel springs diag(k1,k2) q.   q: 2x1 joint angles, p: parameters.   G: 2x1.

G = [p.m1*p.g*p.l1*sin(q(1)) + p.m2*p.g*(p.l1*sin(q(1)) + p.l2*sin(q(1) + q(2)));
     p.m2*p.g*p.l2*sin(q(1) + q(2))];
G = G + diag([p.k1, p.k2])*q;
end

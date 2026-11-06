function Gu = dp_Gu(q1, q2, p)
%DP_GU  Unactuated potential force Gu at the joint angles (q1,q2) (Eq. (17)).
%   Its zero level set is the attainable-equilibria manifold E (Eq. (18)); used by the
%   manifold plots (Figs. 10, 13).
G = dp_gravity([q1; q2], p);
Gu = GuFunction_supp(p.alpha, p.r1, p.r2, G(1), G(2));
end

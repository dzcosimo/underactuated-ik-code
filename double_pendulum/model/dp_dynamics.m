function qdd = dp_dynamics(q, qd, tau, p)
%DP_DYNAMICS  Forward dynamics of the double pendulum, Eq. (4):
%       M(q) qdd + C(q,qd) qd + D qd + G(q) = A tau
%
%   q, qd : 2x1 joint angles (from the downward vertical, relative) and velocities
%   tau   : scalar tendon tension          p : parameters (dp_params)
%   qdd   : 2x1 joint accelerations
%   Point masses at the link tips; same expressions as the original f_fcn/invM_fcn/C_fcn.

c2 = cos(q(2));  s2 = sin(q(2));
M = [p.m1*p.l1^2 + p.m2*(p.l1^2 + 2*p.l1*p.l2*c2 + p.l2^2), p.m2*(p.l2^2 + p.l1*p.l2*c2);
     p.m2*(p.l2^2 + p.l1*p.l2*c2),                          p.m2*p.l2^2];
c = p.m2*p.l1*p.l2;
C = [-c*s2*qd(2), -c*s2*(qd(1) + qd(2));
      c*s2*qd(1),  0];
D = diag([p.d1, p.d2]);
qdd = M \ (-C*qd - dp_gravity(q, p) - D*qd + dp_actuation(p)*tau);
end

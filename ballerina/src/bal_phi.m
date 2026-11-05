function [qu, Huu] = bal_phi(qa, k, qu0)
%BAL_PHI  Statics map of Algorithm 1: qu = phi(qa), the solution of G_u(qa, qu) = 0.
%
%   [qu, Huu] = bal_phi(qa, k, qu0)
%     qa   3x1 actuated coordinates
%     k    pointe stiffness
%     qu0  warm start of the Newton iteration (default 0); the simulation passes the previous qu
%     qu   passive angle at equilibrium;  Huu = H_uu at the solution
%
%   G_u = k qu - M g x_c(qa, qu) (spring torque balances the gravity torque of the CoM about the
%   pointe). Scalar Newton on G_u, with dG_u/dqu = H_uu = k - M g z_c. If H_uu > 0 the
%   equilibrium is stable and locally unique. A warning is issued if Newton does not converge or
%   ends at H_uu <= 0.

if nargin < 3, qu0 = 0; end
qu = qu0;
for it = 1:50
    q = [qa(:); qu]; G = bal_G(q, k); H = bal_H(q, k); Huu = H(4,4);
    d = G(4)/Huu; qu = qu - d;
    if abs(d) < 1e-13, break; end
end
if ~(Huu > 0) || abs(d) > 1e-8
    warning('bal_phi:noconv', 'phi: H_uu = %.3g, last step %.2e', Huu, d);
end
end

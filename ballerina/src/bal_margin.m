function [Huu, schur] = bal_margin(q, k, P)
%BAL_MARGIN  Stability margins at the configuration q (used by summarize_results).
%
%   [Huu, schur] = bal_margin(q, k, P)
%     q      4x1 configuration;  k  pointe stiffness;  P  3x3 low-level joint stiffness
%     Huu    H_uu = k - M g z_c, the unactuated block of the potential Hessian (open loop:
%            the pointe statics is stable iff H_uu > 0)
%     schur  H_uu - H_ua (H_aa + P)^-1 H_au, the Schur complement of the Hessian of the frozen
%            closed loop, H + blkdiag(P, 0). The low-level loop with fixed q_bar is a physical
%            system with potential U + (qa_bar - qa)' P (qa_bar - qa)/2 - G_a(q_bar)' qa; given
%            H_aa + P > 0, schur > 0 <=> its Hessian is positive definite at q (strict minimum).

H = bal_H(q, k); Huu = H(4,4);
if nargin > 2, schur = Huu - H(4,1:3)/(H(1:3,1:3) + P)*H(1:3,4); end
end

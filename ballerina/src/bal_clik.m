function [v, info] = bal_clik(alg, qbar, e, k, Gam, Gp)
%BAL_CLIK  CLIK update (before saturation) for the ballerina: task x = CoM (2x1), n = 4, 3 actuators.
%
%   [v, info] = bal_clik(alg, qbar, e, k, Gam, Gp)
%
%   Inputs
%     alg   1 (Algorithm 1) or 2 (Algorithm 2, non-square generalization)
%     qbar  4x1 CLIK state. Alg. 1: (qa_bar, phi(qa_bar)), on the equilibrium manifold.
%           Alg. 2: free point of R^4.
%     e     2x1 task error, computed by the caller:
%           ff: e = x_ref - h(qbar);   fb: e = x_ref - x(measured q)
%     k     pointe stiffness used by the CLIK model
%     Gam   task gain Gamma (scalar)
%     Gp    statics gain Gamma_perp (scalar, Alg. 2 only)
%   Outputs
%     v     Alg. 1: qa_bar_dot (3x1);  Alg. 2: qbar_dot (4x1)
%     info  .Huu = H_uu(qbar), .sv = singular values of J (for logging)
%
%   Notation (H = d2U/dq2, Jc = [Ja Ju] = CoM Jacobian, G_u = dU/dqu):
%     X = H_uu^-1 H_ua                       (1x3; dqu/dqa along the manifold is -X)
%     J = Ja - Ju X                          (2x3, task Jacobian along the equilibrium manifold)
%   Algorithm 1:
%     qa_bar' = J^+ Gam e
%   Algorithm 2 (non-square generalization):
%     T = [I3; -X],  Y = Ja^+ Ju
%     qbar'   = T J^+ Gam e  -  [-Y; 1] (H_uu - H_ua Y)^-1 Gp G_u(qbar)
%   The first term moves along the manifold tangent and changes only the task; the second term
%   leaves the task unchanged to first order (Jc [-Y; 1] = 0) and drives G_u to zero. With a
%   square, invertible Ja these reduce to the paper's P_E Ja^-1 and P_Z H_uu^-1 terms (see README).
%
%   J^+ is the plain Moore-Penrose pseudoinverse (no damping), computed as J'(J J')^-1 for the
%   full-row-rank 2x3 J. J stays full rank along the main/mismatch runs (sigma_min >= 0.037,
%   logged as R.svmin); the +-1 rad/s saturation (bal_sat) bounds the update near low sigma_min.

Jc = bal_Jcom(qbar); Ja = Jc(:,1:3); Ju = Jc(:,4);
H = bal_H(qbar, k); Huu = H(4,4); Hua = H(4,1:3);
X = Hua/Huu; J = Ja - Ju*X; Jp = J.'/(J*J.');          % pinv of full-row-rank 2x3
if alg == 1
    v = Jp*(Gam*e);
else
    G = bal_G(qbar, k); Y = Ja.'/(Ja*Ja.')*Ju;          % Y = Ja^+ Ju (Ja is 2x3, full row rank)
    v = [eye(3); -X]*(Jp*(Gam*e)) - [-Y; 1]*((Huu - Hua*Y)\(Gp*G(4)));
end
if nargout > 1, info.Huu = Huu; info.sv = svd(J); end
end

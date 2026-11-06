function [qa_dot, qu] = clik_alg1(qa, xbar, p, example, Gamma)
%CLIK_ALG1  Algorithm 1 (Sec. 4, Fig. 6): CLIK in the actuation space with an explicit
%   solution of the statics,  Eq. (33):
%       qa_dot = J_{h o ha}(qa)^-1 Gamma (xbar - h(ha(qa))),   qu = qbar_u(qa)
%   with the end-to-end Jacobian J_{h o ha} = [Ja Ju] [1; dqbar_u/dqa].
%
%   qa : actuated coordinate (CLIK state)   xbar : task reference   p : parameters
%   example : parametrization of the equilibria (see ALG1_PARAMETRIZATION)
%   Gamma   : gain (1 in the paper)
%   qa_dot  : CLIK velocity     qu : unactuated coordinate on the equilibrium manifold

[qu, dqu] = alg1_parametrization(qa, p, example);
T = dp_act_terms(qa, qu, p);
J_e2e = [T.Ja, T.Ju]*[1; dqu];             % end-to-end Jacobian (Fig. 5)
qa_dot = inv(J_e2e)*Gamma*(xbar - T.x);    %#ok<MINV> scalar here; kept as in Eq. (33)
end

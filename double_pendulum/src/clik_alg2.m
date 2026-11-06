function q_act_dot = clik_alg2(q_act, xbar, p, GammaZ, GammaE)
%CLIK_ALG2  Algorithm 2 (Sec. 5, Fig. 8), Eq. (42):
%       d/dt (qa,qu) = P_E Ja^-1 GammaZ (xbar - h(qa,qu)) - P_Z Huu^-1 GammaE Gu(qa,qu)
%
%   q_act  : CLIK state [qa; qu]      xbar : task reference      p : parameters
%   GammaZ : gain of the task term ("Gamma_parallel" in the original code)
%   GammaE : gain of the statics term ("Gamma_perp" in the original code)
%   Near singular Ja or Huu a small diagonal loading is used (REGULARIZED_INVERSE).
%   (Original name: IK_a2.)

T = dp_act_terms(q_act(1), q_act(2), p);
kinematic_inversion = regularized_inverse(T.Ja)*GammaZ*(xbar - T.x);   % standard CLIK
statics_inversion   = regularized_inverse(T.Huu)*GammaE*T.Gu;          % drive Gu -> 0
q_act_dot = projector_PE(T)*kinematic_inversion - projector_PZ(T)*statics_inversion;
end

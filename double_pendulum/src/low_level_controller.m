function tau = low_level_controller(q_act_m, qbar_act, p, Kp, Ga_scale)
%LOW_LEVEL_CONTROLLER  Collocated low-level regulator of Sec. 5.4:
%       tau = Ga_scale * Ga(qa,qu) + Kp (qbar_a - qa)
%   (potential-force cancellation at the measured configuration + P action on the actuated
%    coordinate; Pustina et al. 2022).
%
%   q_act_m  : measured [qa; qu]     qbar_act : CLIK reference [qbar_a; qbar_u]
%   Kp       : proportional gain (10 in Figs. 15-16)
%   Ga_scale : 1 nominal; < 1 models an imperfect cancellation (perturbed case of Fig. 16,
%              where the code used 0.5, see README)
%   The original block also had an integral term with gain 0; it is omitted.
T = dp_act_terms(q_act_m(1), q_act_m(2), p);
tau = Ga_scale*T.Ga + Kp*(qbar_act(1) - q_act_m(1));
end

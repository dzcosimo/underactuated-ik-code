function tau = low_level_controller(q_act, q_act_bar, mdl)
%LOW_LEVEL_CONTROLLER  Actuation-space regulator that tracks the CLIK output q_bar:
%     tau = G_a(q_bar_a, q_bar_u) + Kp (q_bar_a - q_a),   Kp = 10
%   i.e. gravity/elasticity compensation computed on the reference configuration plus a
%   proportional term on the actuated coordinates (paper, low-level controller of the
%   soft-robot simulations).
%   q_act     [n x 1] measured configuration in actuation coordinates [qa; qu]
%   q_act_bar [n x 1] reference configuration [qa_bar; qu_bar] (output of the CLIK law)
%   mdl       model struct (pcc_model.m)
%   tau       [m x 1]
%   The proportional error uses the raw coordinates; G_a is evaluated at the floored reference
%   (floor_curvature.m, applied in body coordinates).
Kp = 10;
m = 2;
err = q_act_bar(1:m) - q_act(1:m);
q_bar = floor_curvature(q_act_to_q(q_act_bar(1:m), q_act_bar(m+1:end)));
q_act_bar = q_to_q_act(q_bar);
tau = Ga_total(q_act_bar(1:m), q_act_bar(m+1:end), mdl) + Kp*err;
end

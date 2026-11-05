function x = forward_kinematics_act(qa, qu, L)
%FORWARD_KINEMATICS_ACT  Tip position in actuation coordinates: x = h(q_act_to_q(qa, qu)).
%   qa [m x 1] actuated coordinates, qu [(n-m) x 1] unactuated coordinates, L segment lengths.
%   Used by the feedforward (ff) versions of the CLIK laws, where the task error is computed on
%   the CLIK state q_bar instead of the measured robot configuration.
q = q_act_to_q(qa(:), qu(:));          % constant linear map, generated (model/generated/)
x = forward_kinematics_planar_PCC(q, 1, L);
end

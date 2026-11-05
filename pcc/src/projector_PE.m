function P_E = projector_PE(q_act, mdl)
%PROJECTOR_PE  Projector P_E(qa, qu) of Eq. (42) (task channel of Alg. 2):
%     P_E = [ I ; -H_uu^-1 H_ua ] (I - J_a^-1 J_u H_uu^-1 H_ua)^-1        [n x m]
%   Its columns are tangent to the static manifold G_u = 0 (H_ua P_E,a + H_uu P_E,u = 0) and
%   J_a P_E,a + J_u P_E,u = J_a, so that P_E J_a^-1 v moves the task with velocity v.
%   q_act [n x 1] = [qa; qu] (m = 2), mdl model struct (pcc_model.m). No regularization.
m = 2;
n = numel(q_act);
qa = q_act(1:m);
qu = q_act(m+1:end);
Ja  = JaFunction(qa, qu);
Ju  = JuFunction(qa, qu);
Huu = Huu_total(qa, qu, mdl);
Hua = Hua_total(qa, qu, mdl);
X = Huu \ Hua;                       % H_uu^-1 H_ua           [(n-m) x m]
M = eye(m) - (Ja \ (Ju*X));          % I - J_a^-1 J_u X       [m x m]
P_E = [eye(m); -X] / M;
assert(all(size(P_E) == [n, m]));
end

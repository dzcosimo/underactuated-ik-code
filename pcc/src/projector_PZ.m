function P_Z = projector_PZ(q_act, mdl)
%PROJECTOR_PZ  Projector P_Z(qa, qu) (statics channel of Alg. 2, companion of Eq. (42)):
%     P_Z = [ -J_a^-1 J_u ; I ] (I - H_uu^-1 H_ua J_a^-1 J_u)^-1          [n x (n-m)]
%   Its columns leave the task unchanged (J_a P_Z,a + J_u P_Z,u = 0) and act on G_u as H_uu, so
%   that -P_Z H_uu^-1 G_u relaxes the statics residual at fixed task.
%   q_act [n x 1] = [qa; qu] (m = 2), mdl model struct (pcc_model.m). No regularization.
m = 2;
n = numel(q_act);
qa = q_act(1:m);
qu = q_act(m+1:end);
Ja  = JaFunction(qa, qu);
Ju  = JuFunction(qa, qu);
Huu = Huu_total(qa, qu, mdl);
Hua = Hua_total(qa, qu, mdl);
Y = Ja \ Ju;                         % J_a^-1 J_u             [m x (n-m)]
X = Huu \ Hua;                       % H_uu^-1 H_ua           [(n-m) x m]
M = eye(n - m) - X*Y;
P_Z = [-Y; eye(n - m)] / M;
assert(all(size(P_Z) == [n, n - m]));
end

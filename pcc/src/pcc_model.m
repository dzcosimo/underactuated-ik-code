function mdl = pcc_model(kscale)
%PCC_MODEL  Model struct passed to every model-dependent function of the simulation.
%
%   mdl = pcc_model(kscale)
%     kscale  stiffness scaling w.r.t. the base robot of model/generated/ (1: stiff robot of
%             Fig. 18, "k200100"; 0.03: soft robot of Fig. 19, "k63")
%   mdl fields
%     p       physical parameters (pcc_params)
%     L       segment lengths (= p.L)
%     kscale  as above
%     dK      [n x n]  stiffness change in body coordinates, (kscale - 1) K0, K0 = diag(p.k)
%     dKa_a, dKa_u, dKa_ua, dKa_uu   blocks of T' dK T in actuation coordinates [qa; qu]
%             (rows a = 1:m, u = m+1:n), T = dq/d[qa; qu] (constant)
%     Kuu     [(n-m) x (n-m)] elastic stiffness of the unactuated coordinates, block uu of
%             T' (kscale K0) T (used by the gravity-expanded statics of Alg. 1, clik_alg1.m)
%
%   Stiffness scaling. The potential force is G(q) = G_grav(q) + K q (q_rest = 0), linear in K.
%   The generated G_fcn, GaFunction_act, GuFunction_act, HuaFunction, HuuFunction contain the base
%   stiffness K0; the functions G_total, Ga_total, Gu_total, Hua_total, Huu_total add the exact
%   correction dK (resp. its blocks) so that no symbolic regeneration is needed for kscale ~= 1.
p = pcc_params();
m = p.m;
T = Jacobiab_q_act_to_q(zeros(m, 1), zeros(p.n - m, 1));   % constant map [qa; qu] -> q
dK  = (kscale - 1)*diag(p.k);
dKa = T.'*dK*T;
K   = kscale*diag(p.k);
Kt  = T.'*K*T;

mdl.p      = p;
mdl.L      = p.L;
mdl.kscale = kscale;
mdl.dK     = dK;
mdl.dKa_a  = dKa(1:m, :);
mdl.dKa_u  = dKa(m+1:end, :);
mdl.dKa_ua = dKa(m+1:end, 1:m);
mdl.dKa_uu = dKa(m+1:end, m+1:end);
mdl.Kuu    = Kt(m+1:end, m+1:end);
end

function T = dp_act_terms(qa, qu, p)
%DP_ACT_TERMS  All terms of Algorithms 1-2 at a point (qa,qu) in actuation coordinates.
%
%   T = dp_act_terms(qa, qu, p) returns a struct with fields
%     q    joint angles q = T (qa;qu)                        (Eq. (16))
%     x    task h(q) (horizontal end-effector position)       (Eq. (7))
%     Ja   dh/dqa,   Ju  dh/dqu                               (task Jacobian blocks, Eq. (43))
%     Ga   actuated potential force, Gu unactuated one        ([Ga;Gu] = T' G, Eq. (17))
%     Hua  dGu/dqa,  Huu dGu/dqu                              (Hessian blocks, Eq. (43))
%   The expressions are the generated functions in model/generated/ (see
%   support_calculate_symbolic.m); they replace the original *Function / *Function_act wrappers.

q = q_act_to_q(qa, qu, p.alpha, p.r1, p.r2);
G = dp_gravity(q, p);
T.q   = q;
T.x   = dp_task(q, p);
T.Ja  = JaFunction_supp(qa, qu, p.alpha, p.r1, p.r2, p.l1, p.l2);
T.Ju  = JuFunction_supp(qa, qu, p.alpha, p.r1, p.r2, p.l1, p.l2);
T.Gu  = GuFunction_supp(p.alpha, p.r1, p.r2, G(1), G(2));
T.Ga  = GaFunction_supp(p.alpha, p.r1, p.r2, G(1), G(2));
T.Hua = HuaFunction_supp(qa, qu, p.alpha, p.r1, p.r2, p.m1, p.m2, p.g, p.l1, p.l2, p.k1, p.k2);
T.Huu = HuuFunction_supp(qa, qu, p.alpha, p.r1, p.r2, p.m1, p.m2, p.g, p.l1, p.l2, p.k1, p.k2);
end

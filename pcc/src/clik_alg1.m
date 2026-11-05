function [qa_dot, qu] = clik_alg1(qa, x_err, statics, mdl)
%CLIK_ALG1  Algorithm 1 of the paper (CLIK on the actuated coordinates through an explicit
%   static branch qu = phi(qa)), with the Jacobian-transpose update:
%     qa_dot = J(qa)' Gamma (x_bar - x),   J(qa) = [J_a, J_u](qa, phi(qa)) [I; dphi/dqa]
%     qu     = phi(qa)
%   Gamma = 4.5 I. J is the end-to-end task Jacobian of the map qa -> h(qa, phi(qa)).
%
%   Inputs
%     qa       [m x 1] actuated coordinates of the CLIK state (q_bar_a)
%     x_err    [m x 1] task error x_bar - x (x from the robot (fb) or from the CLIK state (ff))
%     statics  which static branch phi is used:
%       'zero'   gravity neglected (elastic statics only): phi0(qa) = qBarU(qa), linear in qa,
%                with constant derivative J_qBarU (both generated). This is the Alg. 1 of
%                Figs. 18-19.
%       'first'  statics to FIRST order in gravity around phi0. Writing G_u(qa, qu) =
%                E (qu - qu0) + g N(qa, qu), with qu0 = phi0(qa), E = K_uu (elastic stiffness of
%                the unactuated coordinates; the elastic force vanishes on qu0), and expanding
%                qu = qu0 + g q1 + g^2 q2 + ...:
%                   phi1(qa) = qu0 - E^-1 G_u(qa, qu0)
%       'second' the same expansion to SECOND order in gravity (q2 = -E^-1 dN/dqu(qu0) q1,
%                dN/dqu = H_uu - E):
%                   phi2(qa) = qu0 + (2 I - E^-1 H_uu(qa, qu0)) d1,   d1 = -E^-1 G_u(qa, qu0)
%                Both expansions are closed form (no iterative solver). G_u and H_uu are
%                evaluated at the floored curvature (floor_curvature.m). dphi/dqa is computed by
%                central differences (step 1e-6).
%     mdl      model struct (pcc_model.m; provides Kuu = E and the stiffness scaling)
%   Outputs
%     qa_dot   [m x 1] CLIK velocity of the actuated coordinates (saturated by the caller)
%     qu       [(n-m) x 1] unactuated coordinates of the reference, qu = phi(qa)
qa = qa(:);
x_err = x_err(:);
m = numel(x_err);
Gamma = 1.5*3*eye(m);

if strcmp(statics, 'zero')
    qu = qBarU(qa);
    P = J_qBarU(qa);                                  % d phi0 / d qa (constant)
else
    switch statics
        case 'first',  phi = @(a) phi_first(a, mdl);
        case 'second', phi = @(a) phi_second(a, mdl);
        otherwise, error('clik_alg1: unknown statics ''%s''.', statics);
    end
    qu = phi(qa);
    h = 1e-6;
    P = zeros(numel(qu), m);
    for i = 1:m                                       % central differences of phi
        e = zeros(m, 1); e(i) = h;
        P(:, i) = (phi(qa + e) - phi(qa - e))/(2*h);
    end
end

J = [JaFunction(qa, qu), JuFunction(qa, qu)]*[eye(m); P];
qa_dot = J.'*(Gamma*x_err);
end

function qu = phi_first(qa, mdl)
% static branch to first order in gravity: qu0 - E^-1 G_u(qa, qu0)
qu0 = qBarU(qa);
qu = qu0 - mdl.Kuu \ Gu_floored(qa, qu0, mdl);
end

function qu = phi_second(qa, mdl)
% static branch to second order in gravity: qu0 + (2 I - E^-1 H_uu(qa, qu0)) d1
qu0 = qBarU(qa);
d1 = -(mdl.Kuu \ Gu_floored(qa, qu0, mdl));
qu = qu0 + 2*d1 - mdl.Kuu \ (Huu_floored(qa, qu0, mdl)*d1);
end

function Gu = Gu_floored(qa, qu, mdl)
% G_u at the floored curvature (the generated functions are 0/0 at zero curvature)
qs = q_to_q_act(floor_curvature(q_act_to_q(qa, qu)));
Gu = Gu_total(qs(1:2), qs(3:end), mdl);
end

function H = Huu_floored(qa, qu, mdl)
% H_uu at the same floored point as Gu_floored
qs = q_to_q_act(floor_curvature(q_act_to_q(qa, qu)));
H = Huu_total(qs(1:2), qs(3:end), mdl);
end

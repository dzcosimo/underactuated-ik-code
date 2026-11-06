function [qu, dqu_dqa] = alg1_parametrization(qa, p, example)
%ALG1_PARAMETRIZATION  Closed-form parametrization qu = qbar_u(qa) of the attainable
%   equilibria, needed by Algorithm 1 (Sec. 4.3), and its derivative dqbar_u/dqa.
%
%   example = 'base_actuation'  standard double pendulum actuated at the base
%             (theta = 0, k1 = k2 = 0, g ~= 0):  qu = -qa                       Eq. (28)
%   example = 'no_gravity'      general actuation without gravity (g = 0):
%             qu = (k1-k2) sin(2 theta) / (k1 (1-cos 2theta) + k2 (1+cos 2theta)) qa   Eq. (31)
%   (In the original code this was qBarU.m with a hard-coded switch, and the derivative
%    J_qBarU.m was regenerated symbolically; both maps are linear, so it is written here.)

switch example
    case 'base_actuation'
        c = -1;
    case 'no_gravity'
        th = p.theta;
        c = ((p.k1 - p.k2)*sin(2*th)) / (p.k1*(1 - cos(2*th)) + p.k2*(1 + cos(2*th)));
    otherwise
        error('alg1_parametrization: unknown example "%s"', example);
end
qu = c*qa;
dqu_dqa = c;
end

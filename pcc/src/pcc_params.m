function p = pcc_params()
%PCC_PARAMS  Physical parameters of the planar PCC soft robot (paper, Sec. on the soft-robot
%   simulations, Figs. 18-19).
%
%   p = pcc_params() returns a struct with
%     n, m       number of constant-curvature (CC) segments (5) and of actuators (2)
%     L          [n x 1] segment lengths [m]            (total length 1 m)
%     mass       [n x 1] segment masses [kg]
%     mpts       point masses per segment used to build the inertia M(q)
%     b          [n x 1] viscous damping [N m s/rad]
%     k          [n x 1] bending stiffness [N m/rad] of the BASE robot ("k200100": 200/3 on the
%                three segments of the first actuated section, 100/2 on the two of the second)
%     g, phi_g   gravity acceleration [m/s^2] and direction (pi/2: along the base y axis)
%     q_rest     [n x 1] rest curvatures (straight robot)
%     q0         [n x 1] initial curvatures (both sections bent by -pi/6 in total)
%
%   The values used by model/generate_pcc_model.m to build the symbolic model (M, C, G, D, A, ...)
%   come from here. The functions in model/generated/ were generated with exactly these values;
%   if you change mass, b, k, g, phi_g, L or q_rest you must regenerate them. The stiffness can
%   instead be scaled at run time without regeneration (see pcc_model.m, field kscale): the
%   soft robot of Fig. 19 is the base robot with kscale = 0.03 ("k63").
p.n    = 5;
p.m    = 2;
n      = p.n;
n1     = ceil(n/2);                      % segments of the first actuated section
n2     = floor(n/2);                     % segments of the second actuated section
p.L    = [0.5*ones(n1, 1)/n1; 0.5*ones(n2, 1)/n2];
p.mass = [1*ones(n1, 1)/n1; 1*ones(n2, 1)/n2];
p.mpts = 2;
p.b    = [10*2*ones(n1, 1)/n1; 10*2*ones(n2, 1)/n2];
p.k    = [100*2*ones(n1, 1)/n1; 100*1*ones(n2, 1)/n2];
p.g    = 9.81;
p.phi_g = pi/2;
p.q_rest = zeros(n, 1);
p.q0   = [-pi/6*ones(n1, 1)/n1; -pi/6*ones(n2, 1)/n2];
end

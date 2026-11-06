function opts = default_ode_options()
%DEFAULT_ODE_OPTIONS  ode45 tolerances used by all simulations of this folder.
%   The original Simulink models used ode45 with RelTol 1e-4 (CLIK only) or 1e-3 (closed loop);
%   here tighter tolerances are used, so that the result does not depend on the step-size
%   control of the solver (see README, "Validation").
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
end

function Y = ode_piecewise(rhs, breaks, y0, t_out, opts)
%ODE_PIECEWISE  Integrate y' = rhs(t,y) with ode45, restarting at the time instants where
%   the reference is discontinuous, and return the solution at the requested times.
%
%   rhs    : function handle @(t,y)       breaks : [t0, t1, ..., tf] segment boundaries
%   y0     : initial state (column)       t_out  : output times in [t0, tf] (column)
%   opts   : odeset options
%   Y      : numel(t_out) x numel(y0); at a break the value of the later segment is used
%            (the state is continuous, so this only matters for the reference).
%   Simulink's variable-step ode45 also stops exactly at these instants (step zero crossing /
%   sample hit of the pulse generator).

Y = zeros(numel(t_out), numel(y0));
y = y0(:);
for k = 1:numel(breaks) - 1
    sol = ode45(rhs, [breaks(k), breaks(k+1)], y, opts);
    if k < numel(breaks) - 1
        idx = t_out >= breaks(k) & t_out < breaks(k+1);
    else
        idx = t_out >= breaks(k) & t_out <= breaks(k+1);
    end
    if any(idx), Y(idx, :) = deval(sol, t_out(idx)).'; end
    y = sol.y(:, end);
end
end

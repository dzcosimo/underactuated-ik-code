function f = plot_pendulum_sequence(t, Q, N, t_end, xBounds, yBounds, p)
%PLOT_PENDULUM_SEQUENCE  Robot evolution: N configurations along a trajectory plus the
%   end-effector path (Figs. 7(a,c), 12). Port of plotMultipleDoublePendula.m.
%
%   t, Q    : time vector and configurations (numel(t) x 2)
%   N       : number of drawn configurations (initial, N-2 intermediate, final)
%   t_end   : configurations at linspace(t(1), t_end, N), the last replaced by t(end)
%   xBounds, yBounds : axis limits        p : parameters
f = new_figure([100 100 560 499]);
ax = axes(f); hold(ax, 'on');
ts = linspace(t(1), t_end, N); ts(end) = t(end);
Qs = interp1(t, Q, ts, 'linear');
styles = [3, 2*ones(1, N-2), 1];
h = gobjects(N, 1);
for i = 1:N
    h(i) = plot_pendulum(ax, Qs(i,:), styles(i), p);
end
% end-effector path on a 10x finer (linear) resampling, as in the original
tf = linspace(t(1), t(end), 10*numel(t));
Qf = interp1(t, Q, tf, 'linear');
E = zeros(numel(tf), 2);
for i = 1:numel(tf)
    e = dp_point_position(Qf(i,:), 1, p); E(i,:) = e(1:2);
end
he = plot(ax, E(:,1), E(:,2), 'r-', 'LineWidth', 1);
style_robot_axes(ax, xBounds, yBounds);
legend(ax, [h(1), h(2), h(N), he], {'Initial', 'Intermediate', 'Final', 'End-Effector'}, ...
    'Interpreter', 'latex');
end

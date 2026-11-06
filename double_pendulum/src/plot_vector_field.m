function f = plot_vector_field(p, xbar)
%PLOT_VECTOR_FIELD  Closed-loop vector field of Algorithm 2 (Eq. (42), GammaZ = GammaE = 1)
%   in the joint space, for reference xbar (one panel of Fig. 13). Port of plot_manifolds.m.
%
%   Background: field magnitude, capped to its [5, 95] percentiles; black: streamlines;
%   dotted gray: level sets of the task h (plot_manifold(2) in the original code).

% field on a 101 x 101 grid
q1range = -pi:(pi/50):pi;
q2range = -pi:(pi/50):pi;
[Q1, Q2] = meshgrid(q1range, q2range);
U = zeros(size(Q1)); W = zeros(size(Q1));
Tjac = Jacobian_q_act_to_q(0, 0, p.alpha, p.r1, p.r2);   % dq/d(qa,qu), constant
for i = 1:numel(Q1)
    q_act = q_to_q_act(Q1(i), Q2(i), p.alpha, p.r1, p.r2);
    dq = Tjac*clik_alg2(q_act, xbar, p, 1, 1);            % field mapped back to joint angles
    U(i) = dq(1); W(i) = dq(2);
end

f = new_figure([100 100 560 499]);
ax = axes(f); hold(ax, 'on');
% level sets of the task (as plot_manifold(2): 200 x 200 grid, 15 levels)
[X, Y] = meshgrid(linspace(-pi, pi, 200), linspace(-0.99*pi, 0.99*pi, 200));
Z = arrayfun(@(a, b) dp_task([a; b], p), X, Y);
contour(ax, X, Y, Z, 15, 'LineStyle', ':', 'LineColor', [0.5 0.5 0.5], 'LineWidth', 1);
% capped magnitude
Mag = sqrt(U.^2 + W.^2);
hi = prctile(Mag(:), 95);  lo = prctile(Mag(:), 5);
Mag = max(min(Mag, hi) - lo, 0);
imagesc(ax, q1range, q2range, Mag);
set(ax, 'YDir', 'normal');
hs = streamslice(ax, Q1, Q2, U, W);
set(hs, 'LineWidth', 2, 'Color', 'k');
xlabel(ax, '$q_1$', 'Interpreter', 'latex', 'FontSize', 28);
ylabel(ax, '$q_2$', 'Interpreter', 'latex', 'FontSize', 28);
set(ax, 'FontSize', 24, 'TickLabelInterpreter', 'latex');
box(ax, 'on'); grid(ax, 'off');
axis(ax, [-pi pi -pi pi]);
end

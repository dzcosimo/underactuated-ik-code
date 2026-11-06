function f = plot_config_comparison(p, RR, labels, colors)
%PLOT_CONFIG_COMPARISON  Fig. 10: Algorithm 2 trajectories in the joint space with the task
%   level sets Z (dotted), the level sets of Gu (thin gray) and the attainable-equilibria
%   manifold E = {Gu = 0} (thick), for different gains (GammaZ, GammaE).
%
%   RR     : cell array of SIM_ALG2 results (xbar = 0.5 on [0,10), -0.5 on [10,20])
%   labels : LaTeX gain labels, e.g. '$\Gamma_{\mathcal{E}} = 0$, $\Gamma_{\mathcal{Z}} = 1$'
%   colors : one RGB row per result
%   The figure is the plot_manifold(3) view of the original code (grid q1 in [0, 2pi]),
%   restyled and zoomed as in the paper.

f = new_figure([100 50 856 817]);
ax = axes(f); hold(ax, 'on');
[X, Y] = meshgrid(linspace(0, 2*pi, 200), linspace(-0.99*pi, 0.99*pi, 200));
Zh = arrayfun(@(a, b) dp_task([a; b], p), X, Y);
Zg = arrayfun(@(a, b) dp_Gu(a, b, p), X, Y);
contour(ax, X, Y, Zh, 15, 'LineStyle', ':', 'LineColor', [0.65 0.65 0.65], 'LineWidth', 1.5);
contour(ax, X, Y, Zg, 15, 'LineColor', [0.65 0.65 0.65], 'LineWidth', 1);
contour(ax, X, Y, Zg, [0 0], 'LineColor', [0.15 0.15 0.15], 'LineWidth', 3);
leg = {'$h(q) = \bar{x}$', ...
       '$\mathcal{E}_{\bar{g}_{\mathrm{u}}}$ i.e., $G_{\mathrm{u}}(q) = \bar{g}_{\mathrm{u}}$', ...
       '$\mathcal{E}$', '$q(0)$'};
plot(ax, RR{1}.q(1,1), RR{1}.q(1,2), 'kx', 'LineWidth', 5, 'MarkerSize', 25);
for k = 1:numel(RR)
    i1 = RR{k}.t <= 10;  i2 = RR{k}.t >= 10;          % first / second reference
    plot(ax, RR{k}.q(i1,1), RR{k}.q(i1,2), '--', 'Color', colors(k,:), 'LineWidth', 5);
    plot(ax, RR{k}.q(i2,1), RR{k}.q(i2,2), '-',  'Color', colors(k,:), 'LineWidth', 6 - 3*(k == 2));
    leg = [leg, {['$q([0,10))$, ' labels{k}], ['$q([10,20))$, ' labels{k}]}]; %#ok<AGROW>
end
xlabel(ax, '$q_1$', 'Interpreter', 'latex', 'FontSize', 28);
ylabel(ax, '$q_2$', 'Interpreter', 'latex', 'FontSize', 28);
style_axes(ax, 24);
axis(ax, [1.3649 4.1758 -1.3116 2.5322]);
legend(ax, leg, 'Interpreter', 'latex', 'Position', [0.5012 0.5053 0.4374 0.4669]);
end

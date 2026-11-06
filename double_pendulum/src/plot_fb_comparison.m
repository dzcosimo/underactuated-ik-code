function figs = plot_fb_comparison(RR, scenario)
%PLOT_FB_COMPARISON  Fig. 16: time evolutions of the hierarchical architecture for the three
%   CLIK variants FF, FB task, FB all (one row of Fig. 16).
%
%   RR       : cell {FF, FB task, FB all} of SIM_ALG2_FB results
%   scenario : 'nominal'   -> panels (a) q, (b) qbar, (c) task
%              'perturbed' -> panels (d) q, (e) qbar, (f) task
%   figs     : [f_q, f_qbar, f_x]. Line styles, legends and limits as in the paper.

names  = {'FF', 'FB task', 'FB all'};
colors = {'k', [0 0.447 0.741], [0.851 0.325 0.098]};
nominal = strcmp(scenario, 'nominal');
% per-variant line widths and style of the 2nd coordinate of qbar (nominal: FB task is drawn
% thicker/dotted because it overlaps FF)
lw_qbar = [2 2 2]; ls2_qbar = {'-.', '-.', '-.'}; lw_x = [2 2 2];
if nominal, lw_qbar(2) = 3; ls2_qbar{2} = ':'; lw_x(2) = 3; end
if nominal
    lim = {[-0.9 1.4], [-0.85 0.6], [-1.8 1.2]}; loc = {'northeast', 'southeast', 'southeast'};
else
    lim = {[-0.6 1.3], [-1.4 1.4], [-1.101 1.123]}; loc = {'northeast', 'northwest', 'northeast'};
end

% (a)/(d) robot configuration, (b)/(e) CLIK state
fields = {'q', 'qbar'}; syms = {'q', '\bar{q}'};
figs = gobjects(1, 3);
for k = 1:2
    [figs(k), ax] = new_time_figure();
    leg = {};
    for v = 1:3
        Y = RR{v}.(fields{k});
        if k == 1, lw = 2; ls2 = '-.'; else, lw = lw_qbar(v); ls2 = ls2_qbar{v}; end
        plot(ax, RR{v}.t, Y(:,1), '-', 'Color', colors{v}, 'LineWidth', lw);
        plot(ax, RR{v}.t, Y(:,2), ls2, 'Color', colors{v}, 'LineWidth', lw);
        leg = [leg, {sprintf('$%s_1$ %s', syms{k}, names{v}), sprintf('$%s_2$ %s', syms{k}, names{v})}]; %#ok<AGROW>
    end
    xlabel(ax, 'Time $t$', 'Interpreter', 'latex', 'FontSize', 28);
    ylabel(ax, ['$' syms{k} '$'], 'Interpreter', 'latex', 'FontSize', 28);
    style_axes(ax, 24);
    legend(ax, leg, 'Interpreter', 'latex', 'Location', loc{k});
    ylim(ax, lim{k}); xlim(ax, [0 90]);
end

% (c)/(f) task space; the last sample (t = 90, where the reference switches) is not drawn
[figs(3), ax] = new_time_figure();
n = numel(RR{1}.t) - 1;
plot(ax, RR{1}.t(1:n), RR{1}.xbar(1:n), 'k--', 'LineWidth', 2);
for v = 1:3
    plot(ax, RR{v}.t(1:n), RR{v}.x(1:n), '-', 'Color', colors{v}, 'LineWidth', lw_x(v));
end
xlabel(ax, 'Time $t$', 'Interpreter', 'latex', 'FontSize', 28);
ylabel(ax, 'Task', 'Interpreter', 'latex', 'FontSize', 28);
style_axes(ax, 24);
legend(ax, {'$\bar{x}$', '$x$ FF', '$x$ FB task', '$x$ FB all'}, 'Interpreter', 'latex', 'Location', loc{3});
ylim(ax, lim{3}); xlim(ax, [0 90]);
end

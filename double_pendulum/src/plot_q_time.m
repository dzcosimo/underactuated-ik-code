function f = plot_q_time(t, q, sym)
%PLOT_Q_TIME  Joint angles q1(t) (solid) and q2(t) (dash-dot) (panels "q" of Figs. 7, 11).
%   sym : LaTeX symbol, 'q' (default) or '\bar{q}'.
if nargin < 3, sym = 'q'; end
[f, ax] = new_time_figure();
plot(ax, t, q(:,1), 'k', 'LineWidth', 2);
plot(ax, t, q(:,2), 'k-.', 'LineWidth', 2);
xlabel(ax, 'Time $t$', 'Interpreter', 'latex', 'FontSize', 28);
ylabel(ax, ['$' sym '$'], 'Interpreter', 'latex', 'FontSize', 28);
style_axes(ax, 24);
legend(ax, {['$' sym '_1$'], ['$' sym '_2$']}, 'Interpreter', 'latex');
padded_ylim(ax, q);
end

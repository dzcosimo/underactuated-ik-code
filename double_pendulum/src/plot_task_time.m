function f = plot_task_time(t, x, xbar, legend_location)
%PLOT_TASK_TIME  Task x(t) and reference xbar(t) (panels "Task" of Figs. 7, 11).
%   Port of the first block of plot_results_IK_a1_scheme.m / plot_results_IK_a2_scheme.m.
if nargin < 4, legend_location = 'northeast'; end
[f, ax] = new_time_figure();
plot(ax, t, xbar, 'k--', 'LineWidth', 2);
plot(ax, t, x, 'k', 'LineWidth', 2);
xlabel(ax, 'Time $t$', 'Interpreter', 'latex', 'FontSize', 28);
ylabel(ax, 'Task', 'Interpreter', 'latex', 'FontSize', 28);
style_axes(ax, 24);
legend(ax, {'$\bar{x}$', '$x$'}, 'Interpreter', 'latex', 'Location', legend_location);
padded_ylim(ax, [xbar(:); x(:)]);
end

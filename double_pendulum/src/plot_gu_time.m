function f = plot_gu_time(t, gu)
%PLOT_GU_TIME  Deviation from the attainable-equilibria manifold, Gu(t) (Fig. 11(b)).
[f, ax] = new_time_figure();
plot(ax, t, 0*gu, 'k--', 'LineWidth', 2);
plot(ax, t, gu, 'k', 'LineWidth', 2);
xlabel(ax, 'Time $t$', 'Interpreter', 'latex', 'FontSize', 28);
ylabel(ax, '$G_{\mathrm{u}}$', 'Interpreter', 'latex', 'FontSize', 28);
style_axes(ax, 24);
legend(ax, {'$0$', '$G_{\mathrm{u}}$'}, 'Interpreter', 'latex', 'Location', 'southeast');
padded_ylim(ax, gu);
end

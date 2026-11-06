function f = plot_task_fb(R_ff, R_fb, p)
%PLOT_TASK_FB  Fig. 15: end-effector paths of the robot driven by the FF and the FB (all)
%   variants of Algorithm 2 (nominal case), with the initial (t = 0) and final (t = 90 s)
%   configurations (the final one from the FB run).
f = new_figure([100 100 498 500]);
ax = axes(f); hold(ax, 'on');
h0 = plot_pendulum(ax, R_fb.q(1,:), 3, p);
h1 = plot_pendulum(ax, R_fb.q(end,:), 1, p);
ee = @(R) cell2mat(arrayfun(@(i) dp_point_position(R.q(i,:), 1, p), (1:numel(R.t))', 'UniformOutput', false));
E_fb = ee(R_fb); E_ff = ee(R_ff);
h_fb = plot(ax, E_fb(:,1), E_fb(:,2), '-', 'Color', [0 0.447 0.741], 'LineWidth', 2);
h_ff = plot(ax, E_ff(:,1), E_ff(:,2), 'r-', 'LineWidth', 1.5);
axis(ax, 'equal');
xlabel(ax, 'Horizontal Position', 'Interpreter', 'latex');
ylabel(ax, 'Vertical Position', 'Interpreter', 'latex');
style_axes(ax, 18);
xlim(ax, [-1.6 1.2]); ylim(ax, [-2.222 0.2227]);
legend(ax, [h0, h1, h_fb, h_ff], {'Initial', 'Final', 'End-Effector FB', 'End-Effector FF'}, ...
    'Interpreter', 'latex', 'Location', 'northwest');
end

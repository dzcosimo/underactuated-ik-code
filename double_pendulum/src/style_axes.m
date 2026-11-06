function style_axes(ax, fontsize)
%STYLE_AXES  Common look of the paper figures: LaTeX tick labels, font size, grid, box.
set(ax, 'FontSize', fontsize, 'TickLabelInterpreter', 'latex');
grid(ax, 'on'); box(ax, 'on');
end

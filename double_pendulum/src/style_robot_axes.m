function style_robot_axes(ax, xBounds, yBounds)
%STYLE_ROBOT_AXES  Axes of the robot drawings (Figs. 7(a,c), 12, 15).
axis(ax, 'square');
xlabel(ax, 'Horizontal Position', 'Interpreter', 'latex');
ylabel(ax, 'Vertical Position', 'Interpreter', 'latex');
style_axes(ax, 18);
xlim(ax, xBounds); ylim(ax, yBounds);
end

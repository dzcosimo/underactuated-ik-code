function [f, ax] = new_time_figure()
%NEW_TIME_FIGURE  Figure with the size of the time-evolution panels of the paper (536 x 713 px).
f = new_figure([100 100 536 713]);
ax = axes(f); hold(ax, 'on');
end

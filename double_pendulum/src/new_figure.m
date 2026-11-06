function f = new_figure(position)
%NEW_FIGURE  White figure of the given pixel size [left bottom width height], always in the
%   light theme (MATLAB R2025a and newer follow the dark desktop theme otherwise).
f = figure('Color', 'w', 'Position', position);
if exist('theme', 'file') || exist('theme', 'builtin')
    theme(f, 'light');
end
end

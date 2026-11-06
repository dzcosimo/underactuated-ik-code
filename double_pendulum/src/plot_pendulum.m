function h = plot_pendulum(ax, q, style, p)
%PLOT_PENDULUM  Draw the double pendulum at configuration q.
%   style 1: final configuration (black, with base and joint circles)
%   style 2: intermediate configuration (thin light gray)
%   style 3: initial configuration (dashed dark gray)
%   h : handle of the link line. Port of plotDoublePendulum.m.
numPoints = 100;
P = zeros(numPoints, 2);
for i = 1:numPoints
    pos = dp_point_position(q, (i-1)/(numPoints-1), p);
    P(i, :) = pos(1:2);
end
hold(ax, 'on');
switch style
    case 1
        h = plot(ax, P(:,1), P(:,2), 'k', 'LineWidth', 2);
        plot(ax, P(1,1), P(1,2), 'ko', 'MarkerSize', 7, 'LineWidth', 1.5);           % base
        j = round(numPoints*p.l1/(p.l1 + p.l2));
        plot(ax, P(j,1), P(j,2), 'ko', 'MarkerSize', 7, 'LineWidth', 1.5);           % joint
    case 2
        h = plot(ax, P(:,1), P(:,2), 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
    otherwise
        h = plot(ax, P(:,1), P(:,2), '--', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.5);
end
end

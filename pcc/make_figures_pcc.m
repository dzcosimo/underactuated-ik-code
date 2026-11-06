function make_figures_pcc(varargin)
%MAKE_FIGURES_PCC  Paper Figs. 18 (stiff robot) and 19 (soft robot): the four CLIK laws
%   (Alg. 1 / Alg. 2, feedforward / feedback) reaching the target (0.6, 0.6).
%
%   make_figures_pcc()
%   make_figures_pcc('resdir', dir, 'outdir', dir, 'figs', {'fig18', 'fig19'})
%
%   Reads results/<case>.mat (cases stiff_a1ff ... stiff_a2fb for Fig. 18, soft_a1ff ...
%   soft_a2fb for Fig. 19; produce them with run_pcc_case or run_all_pcc) and writes
%   figures/fig18_stiff.{pdf,png} and figures/fig19_soft.{pdf,png}.
%
%   Layout (as in the paper): left 2 x 2, one panel per law: tip path (Alg. 1 dark gray,
%   Alg. 2 red; ff dashed, fb solid), gray intermediate robot shapes equispaced along the tip
%   path, final robot (black backbone; teal dots at the base, at the end of the first actuated
%   section and at the tip), start (teal square) and goal (red cross); same axes for the four
%   panels. Right: x1(t) and x2(t) of the four laws with the goal (dotted).
root = setup_paths();
ip = inputParser;
ip.addParameter('resdir', fullfile(root, 'results'));
ip.addParameter('outdir', fullfile(root, 'figures'));
ip.addParameter('figs', {'fig18', 'fig19'});
ip.parse(varargin{:});
if ~exist(ip.Results.outdir, 'dir'), mkdir(ip.Results.outdir); end

FIG.fig18 = struct('robot', 'stiff', 'file', 'fig18_stiff');
FIG.fig19 = struct('robot', 'soft', 'file', 'fig19_soft');
target = [0.6; 0.6];
tags = {'a1ff', 'a1fb', 'a2ff', 'a2fb'};
lab  = struct('a1ff', 'Algorithm 1 (ff)', 'a1fb', 'Algorithm 1 (fb)', 'a2ff', 'Algorithm 2 (ff)', 'a2fb', 'Algorithm 2 (fb)');
col  = struct('a1ff', [0.2 0.2 0.2], 'a1fb', [0.2 0.2 0.2], 'a2ff', [0.8 0 0], 'a2fb', [0.6 0 0]);
lsty = struct('a1ff', '--', 'a1fb', '-', 'a2ff', '--', 'a2fb', '-');
nShapes = struct('a1ff', 8, 'a1fb', 10, 'a2ff', 8, 'a2fb', 14);   % gray shapes per panel
maxSamples = 800;                                                  % plotted samples per run
compC = [0 0.6 0.6];
% panel positions (normalized): robot panels 2 x 2 (row 1: Alg. 1, row 2: Alg. 2), time panels
posR = struct('a1ff', [0.07 0.54 0.23 0.41], 'a1fb', [0.32 0.54 0.23 0.41], ...
              'a2ff', [0.07 0.09 0.23 0.41], 'a2fb', [0.32 0.09 0.23 0.41]);
posT = {[0.61 0.54 0.32 0.41], [0.61 0.09 0.32 0.41]};

for fname = ip.Results.figs
    F = FIG.(fname{1});
    % ---- load and decimate the four runs
    R = struct();
    for k = 1:numel(tags)
        f = fullfile(ip.Results.resdir, sprintf('%s_%s.mat', F.robot, tags{k}));
        if ~exist(f, 'file'), error('make_figures_pcc: missing %s (run run_pcc_case(''%s_%s'')).', f, F.robot, tags{k}); end
        S = load(f, 'R');
        idx = unique([1:max(1, floor(numel(S.R.t)/maxSamples)):numel(S.R.t), numel(S.R.t)]);   % keep the last sample
        R.(tags{k}) = struct('t', S.R.t(idx), 'Q', S.R.Q(idx, :), 'tip', S.R.tip(:, idx), 'L', S.R.L);
    end
    % ---- common square limits: all tip paths, base and goal, 5 % margin
    A = [cell2mat(cellfun(@(g) R.(g).tip, tags, 'UniformOutput', false)), [0; 0], target];
    c = (min(A, [], 2) + max(A, [], 2))/2;
    hr = 1.05*max(max(A, [], 2) - min(A, [], 2))/2;
    XL = c(1) + hr*[-1 1]; YL = c(2) + hr*[-1 1];

    fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', 'Position', [100 100 1352 656]);
    theme(fig, 'light');
    % ---- robot panels
    for k = 1:numel(tags)
        g = tags{k}; r = R.(g);
        ax = axes(fig, 'Position', posR.(g)); hold(ax, 'on');
        set(ax, 'XLim', XL, 'YLim', YL, 'DataAspectRatio', [1 1 1], 'TickLabelInterpreter', 'latex', ...
            'FontSize', 12, 'LineWidth', 1, 'Box', 'on', 'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.2);
        % shapes equispaced along the tip path: the last one is the final robot
        s = [0, cumsum(sqrt(sum(diff(r.tip, 1, 2).^2, 1)))];
        idx = unique(arrayfun(@(sv) find(s >= sv, 1, 'first'), linspace(0, s(end), nShapes.(g) + 1)));
        for j = idx(1:end-1)
            bb = robot_backbone(r.Q(j, :).', r.L);
            plot(ax, bb(1, :), bb(2, :), 'Color', [0.6 0.6 0.6 0.2], 'LineWidth', 1);
        end
        plot(ax, r.tip(1, :), r.tip(2, :), 'Color', col.(g), 'LineStyle', lsty.(g), 'LineWidth', 1.4);
        draw_robot(ax, r.Q(idx(end), :).', r.L, compC);
        plot(ax, r.tip(1, 1), r.tip(2, 1), 's', 'MarkerSize', 8, 'MarkerFaceColor', compC, 'MarkerEdgeColor', compC);
        plot(ax, target(1), target(2), 'rx', 'MarkerSize', 14, 'LineWidth', 2);
        text(ax, XL(2) - 0.03*diff(XL), YL(1) + 0.03*diff(YL), lab.(g), 'Interpreter', 'latex', ...
            'FontSize', 13, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
        if g(2) == '1', set(ax, 'XTickLabel', []); else, xlabel(ax, '$x_1$', 'Interpreter', 'latex', 'FontSize', 15); end
        if strcmp(g(3:4), 'fb'), set(ax, 'YTickLabel', []); else, ylabel(ax, '$x_2$', 'Interpreter', 'latex', 'FontSize', 15); end
    end
    % ---- time panels x1(t), x2(t)
    T = R.a1ff.t(end);
    for d = 1:2
        ax = axes(fig, 'Position', posT{d}); hold(ax, 'on');
        set(ax, 'XLim', [0 T], 'TickLabelInterpreter', 'latex', 'FontSize', 12, 'LineWidth', 1, ...
            'Box', 'on', 'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.2);
        hG = plot(ax, [0 T], target(d)*[1 1], 'k:', 'LineWidth', 1.2);
        H = gobjects(1, numel(tags));
        for k = 1:numel(tags)
            g = tags{k};
            H(k) = plot(ax, R.(g).t, R.(g).tip(d, :), 'Color', col.(g), 'LineStyle', lsty.(g), 'LineWidth', 1.2);
        end
        Y = [cell2mat(cellfun(@(g) R.(g).tip(d, :), tags, 'UniformOutput', false)), target(d)];
        ylim(ax, [min(Y), max(Y)] + 0.05*(max(Y) - min(Y))*[-1 1]);
        ylabel(ax, sprintf('$x_%d(t)$', d), 'Interpreter', 'latex', 'FontSize', 15);
        if d == 1
            set(ax, 'XTickLabel', []);
            legend(ax, [hG, H], [{'Goal'}, cellfun(@(g) lab.(g), tags, 'UniformOutput', false)], ...
                'Interpreter', 'latex', 'Location', 'southeast');
        else
            xlabel(ax, 'Time $t$', 'Interpreter', 'latex', 'FontSize', 15);
        end
    end
    out = fullfile(ip.Results.outdir, F.file);
    set(fig, 'PaperUnits', 'inches', 'PaperPosition', [0 0 14.0833 6.8333], 'PaperSize', [14.0833 6.8333]);
    print(fig, [out '.pdf'], '-dpdf', '-vector');
    print(fig, [out '.png'], '-dpng', '-r150');
    close(fig);
    fprintf('written %s.pdf / .png\n', out);
end
end

function draw_robot(ax, q, L, compC)
% final robot: black backbone, gray dots at the segment ends, teal dots at the base, at the end
% of the first actuated section and at the tip
[bb, ee] = robot_backbone(q, L);
mid = ceil(numel(q)/2) + 1;
plot(ax, bb(1, :), bb(2, :), 'k', 'LineWidth', 1.5);
plot(ax, ee(1, :), ee(2, :), '.', 'Color', [0.25 0.25 0.25], 'MarkerSize', 10);
plot(ax, ee(1, [1 mid end]), ee(2, [1 mid end]), '.', 'Color', compC, 'MarkerSize', 22);
end

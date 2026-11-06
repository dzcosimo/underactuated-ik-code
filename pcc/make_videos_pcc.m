function make_videos_pcc(varargin)
%MAKE_VIDEOS_PCC  Real-time videos of the PCC simulations (1 s of video = 1 s of simulated
%   time, 30 frames per second), one MPEG-4 file per panel.
%
%   make_videos_pcc()
%   make_videos_pcc('sets', {'fig18_stiff', ...}, 'resdir', dir, 'outdir', dir, 'maxframes', N)
%
%   Video sets (folder videos/<set>/) and the cases they read from results/:
%     fig18_stiff                 stiff_{a1ff,a1fb,a2ff,a2fb}                    (Fig. 18)
%     fig19_soft                  soft_{a1ff,a1fb,a2ff,a2fb}                     (Fig. 19)
%     fig19_soft_first            robot panels soft_a1{ff,fb}_first; time panels with the
%                                 four laws of Fig. 19 as well
%     fig19_soft_second           robot panels soft_a1{ff,fb}_second; time panels with all
%                                 eight soft single-target runs
%     fig18_stiff_triangle        stiff_tri_{a1ff,a1fb,a2ff,a2fb}
%     fig19_soft_triangle         soft_tri_{a1ff,a1fb,a2ff,a2fb,a1ff_first,a1fb_first}
%     fig19_soft_triangle_second  robot panels soft_tri_a1{ff,fb}_second; time panels with all
%                                 eight soft triangle runs
%   Options: 'sets' (default: all), 'resdir' (default results/), 'outdir' (default videos/;
%   NOTE: overwrites the shipped videos), 'maxframes' (render only the first N frames, for a
%   quick test; also writes the last rendered frame of each panel as a PNG next to the video).
%
%   Robot panels (<law>.mp4): the robot at time t (black backbone; teal dots at the base, at the
%   end of the first actuated section and at the tip; gray dots at the segment ends), the tip
%   path up to t, the gray intermediate shapes (equispaced along the whole tip path) appearing
%   when the robot passes them, the start (teal square) and the target (red cross). With several
%   targets: the active one bold red, the others faded, and the current (sliding) reference as
%   a red dot. Time panels (x1_t.mp4, x2_t.mp4): the tip coordinates of the runs, drawn up to t,
%   and the reference (dotted).
root = setup_paths();
ip = inputParser;
ip.addParameter('sets', {});
ip.addParameter('resdir', fullfile(root, 'results'));
ip.addParameter('outdir', fullfile(root, 'videos'));
ip.addParameter('maxframes', Inf);
ip.parse(varargin{:});
FPS = 30;

lab = struct('a1ff', 'Algorithm 1 (ff)', 'a1fb', 'Algorithm 1 (fb)', 'a2ff', 'Algorithm 2 (ff)', 'a2fb', 'Algorithm 2 (fb)', ...
    'a1ff_first', 'Algorithm 1, $g$ to 1st order (ff)', 'a1fb_first', 'Algorithm 1, $g$ to 1st order (fb)', ...
    'a1ff_second', 'Algorithm 1, $g$ to 2nd order (ff)', 'a1fb_second', 'Algorithm 1, $g$ to 2nd order (fb)');
col = struct('a1ff', [0.2 0.2 0.2], 'a1fb', [0.2 0.2 0.2], 'a2ff', [0.75 0 0], 'a2fb', [0.6 0 0], ...
    'a1ff_first', [0 0.35 0.75], 'a1fb_first', [0 0.35 0.75], 'a1ff_second', [0 0.55 0.25], 'a1fb_second', [0 0.55 0.25]);
lsty = struct('a1ff', '--', 'a1fb', '-', 'a2ff', '--', 'a2fb', '-', ...
    'a1ff_first', '--', 'a1fb_first', '-', 'a1ff_second', '--', 'a1fb_second', '-');
compC = [0 0.6 0.6]; smallC = [0.25 0.25 0.25];

% ---- video sets: name, case prefix, robot panels, time-panel curves, number of gray shapes,
%      legend locations (x1, x2) and font size
L4 = {'a1ff', 'a1fb', 'a2ff', 'a2fb'};
L6 = {'a1ff', 'a1fb', 'a1ff_first', 'a1fb_first', 'a2ff', 'a2fb'};
L8 = {'a1ff', 'a1fb', 'a1ff_first', 'a1fb_first', 'a1ff_second', 'a1fb_second', 'a2ff', 'a2fb'};
G4 = struct('a1ff', 8, 'a1fb', 10, 'a2ff', 8, 'a2fb', 14);       % as in the paper figure
SETS = struct( ...
    'name',   {'fig18_stiff', 'fig19_soft', 'fig19_soft_first', 'fig19_soft_second', ...
               'fig18_stiff_triangle', 'fig19_soft_triangle', 'fig19_soft_triangle_second'}, ...
    'prefix', {'stiff', 'soft', 'soft', 'soft', 'stiff_tri', 'soft_tri', 'soft_tri'}, ...
    'panels', {L4, L4, {'a1ff_first', 'a1fb_first'}, {'a1ff_second', 'a1fb_second'}, L4, ...
               [L4, {'a1ff_first', 'a1fb_first'}], {'a1ff_second', 'a1fb_second'}}, ...
    'curves', {L4, L4, L6, L8, L4, L6, L8}, ...
    'ghosts', {G4, G4, 10, 10, 16, 16, 16}, ...
    'legloc', {{'southeast', 'northeast'}, {'southeast', 'northeast'}, {'best', 'best'}, {'best', 'best'}, ...
               {'best', 'best'}, {'best', 'best'}, {'best', 'best'}}, ...
    'legfs',  {14, 14, 12, 12, 12, 12, 12});
if ~isempty(ip.Results.sets), SETS = SETS(ismember({SETS.name}, ip.Results.sets)); end

for F = SETS
    odir = fullfile(ip.Results.outdir, F.name);
    if ~exist(odir, 'dir'), mkdir(odir); end
    % ---- the runs, resampled on the video clock (frame j at t = (j-1)/FPS)
    R = struct(); tags = unique([F.panels, F.curves], 'stable');
    for g = tags
        f = fullfile(ip.Results.resdir, sprintf('%s_%s.mat', F.prefix, g{1}));
        if ~exist(f, 'file'), error('make_videos_pcc: missing %s (run run_pcc_case(''%s_%s'')).', f, F.prefix, g{1}); end
        S = load(f, 'R');
        t = S.R.t; L = S.R.L; o = S.R.o;
        tv = (0:floor(t(end)*FPS + 1e-6)).'/FPS;
        Qv = interp1(t, S.R.Q, tv, 'linear');
        tip = zeros(2, numel(tv));
        for j = 1:numel(tv), tip(:, j) = forward_kinematics_planar_PCC(Qv(j, :).', 1, L); end
        tb = [o.tswitch, Inf];
        act = arrayfun(@(s) find(s >= tb(1:end-1), 1, 'last'), tv);              % active target
        ref = interp1(t, S.R.xbar, tv, 'previous', 'extrap').';                   % reference used
        % gray shapes: equispaced along the tip path (the last point is the final robot)
        s = [0, cumsum(sqrt(sum(diff(tip, 1, 2).^2, 1)))];
        ng = F.ghosts; if isstruct(ng), ng = ng.(g{1}); end
        ghost = unique(arrayfun(@(sv) find(s >= sv, 1, 'first'), linspace(0, s(end), ng)));
        ghost = ghost(1:end-1);
        R.(g{1}) = struct('t', tv, 'Q', Qv, 'tip', tip, 'L', L, 'xb', o.targets, 'act', act, 'ref', ref, ...
            'ghost', ghost, 'tipfull', tip, 'reffull', ref, 'tfull', tv);
        % quick test: keep only the first maxframes frames (limits and shapes use the full run)
        nf = min(numel(tv), ip.Results.maxframes);
        for fld = {'t', 'Q', 'act'}, R.(g{1}).(fld{1}) = R.(g{1}).(fld{1})(1:nf, :); end
        for fld = {'tip', 'ref'}, R.(g{1}).(fld{1}) = R.(g{1}).(fld{1})(:, 1:nf); end
    end
    n = size(R.(tags{1}).Q, 2); mid = ceil(n/2) + 1;
    X = R.(tags{1}).xb; multi = size(X, 2) > 1;
    % common square limits: all tip paths, base and targets; 5 % margin
    A = [cell2mat(cellfun(@(g) R.(g).tipfull, tags, 'UniformOutput', false)), [0; 0], X];
    c = (min(A, [], 2) + max(A, [], 2))/2; hr = 1.05*max(max(A, [], 2) - min(A, [], 2))/2;
    XL = c(1) + hr*[-1 1]; YL = c(2) + hr*[-1 1];

    % ---- robot panels
    for g = F.panels
        r = R.(g{1}); N = numel(r.t);
        fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', 'Position', [100 100 720 720]); theme(fig, 'light');
        ax = axes(fig, 'Units', 'pixels', 'Position', [95 85 600 600]); hold(ax, 'on'); grid(ax, 'on');
        set(ax, 'XLim', XL, 'YLim', YL, 'DataAspectRatio', [1 1 1], 'TickLabelInterpreter', 'latex', 'FontSize', 16, ...
            'LineWidth', 1, 'Box', 'on', 'GridAlpha', 0.2);
        xlabel(ax, '$x_1$', 'Interpreter', 'latex', 'FontSize', 20); ylabel(ax, '$x_2$', 'Interpreter', 'latex', 'FontSize', 20);
        hTr = plot(ax, NaN, NaN, 'Color', col.(g{1}), 'LineStyle', lsty.(g{1}), 'LineWidth', 1.4);
        [bb, ee] = robot_backbone(r.Q(1, :).', r.L);
        hBb = plot(ax, bb(1, :), bb(2, :), 'k', 'LineWidth', 1.5);
        hSm = plot(ax, ee(1, :), ee(2, :), '.', 'Color', smallC, 'MarkerSize', 10);
        hHi = plot(ax, ee(1, [1 mid end]), ee(2, [1 mid end]), '.', 'Color', compC, 'MarkerSize', 22);
        plot(ax, r.tip(1, 1), r.tip(2, 1), 's', 'MarkerSize', 8, 'MarkerFaceColor', compC, 'MarkerEdgeColor', compC);
        hX = gobjects(1, size(X, 2));
        for i = 1:size(X, 2), hX(i) = plot(ax, X(1, i), X(2, i), 'rx', 'MarkerSize', 14, 'LineWidth', 2); end
        if multi
            hRef = plot(ax, r.ref(1, 1), r.ref(2, 1), 'o', 'MarkerSize', 7, 'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', [1 0 0]);
        end
        text(ax, XL(2) - 0.03*diff(XL), YL(1) + 0.04*diff(YL), lab.(g{1}), 'Interpreter', 'latex', 'FontSize', 18, ...
            'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'BackgroundColor', 'w');
        hT = text(ax, XL(1) + 0.03*diff(XL), YL(2) - 0.04*diff(YL), '', 'Interpreter', 'latex', 'FontSize', 16, ...
            'VerticalAlignment', 'top', 'BackgroundColor', 'w');
        vw = VideoWriter(fullfile(odir, [g{1} '.mp4']), 'MPEG-4'); vw.FrameRate = FPS; vw.Quality = 95; open(vw);
        for j = 1:N
            if any(r.ghost == j)                                 % a gray shape, kept from now on
                bg = robot_backbone(r.Q(j, :).', r.L);
                h = plot(ax, bg(1, :), bg(2, :), 'Color', [0.6 0.6 0.6 0.2], 'LineWidth', 1); uistack(h, 'bottom');
            end
            [bb, ee] = robot_backbone(r.Q(j, :).', r.L);
            set(hTr, 'XData', r.tip(1, 1:j), 'YData', r.tip(2, 1:j));
            set(hBb, 'XData', bb(1, :), 'YData', bb(2, :)); set(hSm, 'XData', ee(1, :), 'YData', ee(2, :));
            set(hHi, 'XData', ee(1, [1 mid end]), 'YData', ee(2, [1 mid end]));
            if multi                                             % active target bold, others faded
                for i = 1:numel(hX)
                    if i == r.act(j), set(hX(i), 'Color', [1 0 0], 'MarkerSize', 14); else, set(hX(i), 'Color', [1 0.7 0.7], 'MarkerSize', 11); end
                end
                set(hRef, 'XData', r.ref(1, j), 'YData', r.ref(2, j));
            end
            set(hT, 'String', sprintf('$t = %.1f$ s', r.t(j)));
            im = frame(fig);
            writeVideo(vw, im);
        end
        close(vw); close(fig);
        if isfinite(ip.Results.maxframes), imwrite(im, fullfile(odir, [g{1} '_lastframe.png'])); end
        fprintf('%s/%s.mp4: %d frames (%.1f s)\n', F.name, g{1}, N, (N - 1)/FPS);
    end

    % ---- time panels x1(t), x2(t)
    r0 = R.(F.curves{1}); N = numel(r0.t); T = r0.tfull(end);
    for d = 1:2
        Y = cell2mat(cellfun(@(g) R.(g).tipfull(d, :), F.curves, 'UniformOutput', false));
        yl = [min([Y, r0.reffull(d, :)]), max([Y, r0.reffull(d, :)])]; yl = yl + 0.05*diff(yl)*[-1 1];
        fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', 'Position', [100 100 960 480]); theme(fig, 'light');
        ax = axes(fig, 'Units', 'pixels', 'Position', [100 80 830 370]); hold(ax, 'on'); grid(ax, 'on');
        set(ax, 'XLim', [0 T], 'YLim', yl, 'TickLabelInterpreter', 'latex', 'FontSize', 16, 'LineWidth', 1, ...
            'Box', 'on', 'GridAlpha', 0.2);
        xlabel(ax, 'Time $t$', 'Interpreter', 'latex', 'FontSize', 20);
        ylabel(ax, sprintf('$x_%d(t)$', d), 'Interpreter', 'latex', 'FontSize', 20);
        hG = plot(ax, r0.tfull, r0.reffull(d, :), 'k:', 'LineWidth', 1.2);   % reference (goal)
        H = gobjects(1, numel(F.curves));
        for k = 1:numel(F.curves)
            H(k) = plot(ax, NaN, NaN, 'Color', col.(F.curves{k}), 'LineStyle', lsty.(F.curves{k}), 'LineWidth', 1.5);
        end
        legend(ax, [hG, H], [{'Goal'}, cellfun(@(g) lab.(g), F.curves, 'UniformOutput', false)], ...
            'Interpreter', 'latex', 'Location', F.legloc{d}, 'FontSize', F.legfs);
        vw = VideoWriter(fullfile(odir, sprintf('x%d_t.mp4', d)), 'MPEG-4'); vw.FrameRate = FPS; vw.Quality = 95; open(vw);
        for j = 1:N
            for k = 1:numel(F.curves)
                rk = R.(F.curves{k}); jk = min(j, numel(rk.t));
                set(H(k), 'XData', rk.t(1:jk), 'YData', rk.tip(d, 1:jk));
            end
            im = frame(fig);
            writeVideo(vw, im);
        end
        close(vw); close(fig);
        if isfinite(ip.Results.maxframes), imwrite(im, fullfile(odir, sprintf('x%d_t_lastframe.png', d))); end
        fprintf('%s/x%d_t.mp4: %d frames (%.1f s)\n', F.name, d, N, (N - 1)/FPS);
    end
end
end

function im = frame(fig)
% the figure as an RGB image with even width and height (required by H.264)
im = print(fig, '-RGBImage', '-r96');
im = im(1:2*floor(size(im, 1)/2), 1:2*floor(size(im, 2)/2), :);
end

function make_videos_ballerina(varargin)
%MAKE_VIDEOS_BALLERINA  Real-time videos (1 s of video = 1 s of simulated time, 30 fps, MPEG-4)
%   of the saved ballerina runs (results/*.mat; run run_all_ballerina first).
%
%   make_videos_ballerina()                    render everything into videos/
%   make_videos_ballerina(name, value, ...)    options:
%     'sets'    cell of set names to render, subset of {'main', 'mismatch', 'soft'} (default all)
%     'only'    cell of panel names to render, e.g. {'a1ff', 'xc'} (default all)
%     'nmax'    render only the first nmax frames of every video (quick check; default all)
%     'outdir'  output folder (default <ballerina>/videos); the files go to <outdir>/<set>/
%   Example (quick test that does not overwrite the shipped videos):
%     make_videos_ballerina('sets', {'main'}, 'only', {'a2ff'}, 'nmax', 60, 'outdir', tempdir)
%
%   Output per set:
%     main/      a1ff, a1fb, a2ff, a2fb robot panels; xc_t, zc_t time panels (all four laws and
%                the reference)
%     mismatch/  a2ff, a2fb robot panels (plant k, model 0.8 k); err_t time panel |x_ref - x|
%     soft/      a2ff robot panel (k_soft: T2 is an unstable equilibrium of the model)
%   Robot panels: the dancer at t (thick rods, joints, head), the pointe spring + damper glyph, the
%   floor, a dashed vertical through the pointe, the CoM (filled, law color) and its trace, the
%   start CoM (teal square), the targets (red crosses; the active one bold, the others faded), the
%   current sliding reference (red dot), gray ghost poses once passed. In the mismatch and soft
%   panels the model CoM h(q_bar) is also shown (hollow marker), to separate the CLIK error from
%   the plant/model error. Figures are rendered off screen (requires R2025a or later: theme()).

root = fileparts(mfilename('fullpath')); addpath(fullfile(root, 'src'), fullfile(root, 'model'));
opt = struct('sets', {{'main', 'mismatch', 'soft'}}, 'only', {{}}, 'nmax', NaN, 'outdir', fullfile(root, 'videos'));
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i+1}; end
FPS = 30; RES = fullfile(root, 'results'); NMAX = opt.nmax; ONLY = opt.only;
lab = struct('a1ff', 'Algorithm 1 (ff)', 'a1fb', 'Algorithm 1 (fb)', 'a2ff', 'Algorithm 2 (ff)', 'a2fb', 'Algorithm 2 (fb)');
col = struct('a1ff', [0 0.45 0.80], 'a1fb', [0 0.25 0.55], 'a2ff', [0.90 0.50 0], 'a2fb', [0.55 0.30 0]);
lsty = struct('a1ff', '--', 'a1fb', '-', 'a2ff', '--', 'a2fb', '-');
SETS = struct('name', {'main', 'mismatch', 'soft'}, ...
    'runs', {{'main_a1ff', 'main_a1fb', 'main_a2ff', 'main_a2fb'}, {'mismatch_a2ff', 'mismatch_a2fb'}, {'soft_a2ff'}}, ...
    'xbar', {false, true, true}, 'tpanels', {{'xc', 'zc'}, {'err'}, {}}, ...
    'note', {'', '(plant $k$, model $0.8\,k$)', '(soft pointe, $k = 120$ Nm/rad)'});
SETS = SETS(ismember({SETS.name}, opt.sets));
compC = [0 0.6 0.6]; bodyC = [0.15 0.15 0.15]; ghostC = [0.55 0.55 0.55 0.18];

for F = SETS
    odir = fullfile(opt.outdir, F.name); if ~exist(odir, 'dir'), mkdir(odir); end
    % ---- the runs, resampled on the video clock
    R = struct(); Pall = zeros(2, 0);
    for g = F.runs
        S = load(fullfile(RES, [g{1} '.mat']), 'R'); s = S.R; p = s.params;
        tv = (0:floor(s.t(end)*FPS + 1e-6)).'/FPS;
        Q = interp1(s.t, s.q, tv); X = interp1(s.t, s.x, tv).'; Xr = interp1(s.t, s.x_ref, tv).';
        Xb = interp1(s.t, s.x_bar, tv).';
        tb = (0:size(s.targets, 2) - 1)*s.opts.hold; act = arrayfun(@(u) find(u >= tb, 1, 'last'), tv);   % active target
        P = pts(Q, p); law = s.law;
        R.(law) = struct('t', tv, 'Q', Q, 'P', P, 'x', X, 'xr', Xr, 'xb', Xb, 'tg', s.targets, 'act', act, 'p', p, ...
                         'err', 1e3*sqrt(sum((Xr - X).^2, 1)));
        Pall = [Pall, reshape(P, 2, [])]; %#ok<AGROW>
    end
    laws = fieldnames(R).'; p = R.(laws{1}).p;
    % ---- common square view: everything drawn (dancer + head + glyph), floor band below z = 0
    A = [Pall, R.(laws{1}).tg, [-0.3 0.3; 0 0]];
    lo = min(A, [], 2) - p.rh - 0.05; hi = max(A, [], 2) + p.rh + 0.05;
    c = (lo + hi)/2; hr = max(hi - lo)/2;
    fb = 0.30 + 0.12*~isempty(F.note); H = max(2*hr, hi(2) + fb); YL = -fb + [0, H]; XL = c(1) + H/2*[-1 1];   % floor band (room for the law label) at the bottom

    % ---- robot panels
    for g = laws
        if ~isempty(ONLY) && ~ismember(g{1}, ONLY), continue; end
        r = R.(g{1}); N = numel(r.t); if ~isnan(NMAX), N = min(N, NMAX); end; C = col.(g{1});
        % ghosts: the pose at the end of every hold, plus a pose every 1 s when it differs enough
        % (summed displacement of the 5 drawn points > 0.5 m) from the last ghost
        hEnd = arrayfun(@(u) find(r.t <= u - 1/FPS, 1, 'last'), (1:size(r.tg, 2) - 1)*(r.t(end)/size(r.tg, 2)));
        ghost = []; last = r.P(:, :, 1);
        for j = 2:numel(r.t) - 1
            if ismember(j, hEnd) || (mod(round(r.t(j)*FPS), FPS) == 0 && sum(vecnorm(r.P(:, 2:6, j) - last(:, 2:6))) > 0.5)
                ghost(end + 1) = j; last = r.P(:, :, j); %#ok<AGROW>
            end
        end
        fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', 'Position', [100 100 720 720]); theme(fig, 'light');
        ax = axes(fig, 'Units', 'pixels', 'Position', [95 85 600 600]); hold(ax, 'on'); grid(ax, 'on');
        set(ax, 'XLim', XL, 'YLim', YL, 'DataAspectRatio', [1 1 1], 'TickLabelInterpreter', 'latex', 'FontSize', 16, ...
            'LineWidth', 1, 'Box', 'on', 'GridAlpha', 0.2, 'Layer', 'top', 'Color', 'w');
        xlabel(ax, '$x$ [m]', 'Interpreter', 'latex', 'FontSize', 20); ylabel(ax, '$z$ [m]', 'Interpreter', 'latex', 'FontSize', 20);
        % floor, hatching, vertical through the pointe, spring/damper posts
        patch(ax, XL([1 2 2 1]), [YL(1) YL(1) 0 0], [0.93 0.93 0.93], 'EdgeColor', 'none');
        hx = XL(1):0.08:XL(2); plot(ax, [hx; hx - 0.07], [zeros(size(hx)); -0.07*ones(size(hx))], 'Color', [0.55 0.55 0.55], 'LineWidth', 0.8);
        plot(ax, XL, [0 0], 'k', 'LineWidth', 1.5);
        plot(ax, [0 0], [0 YL(2)], '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1);
        gp = glyph(0, p); plot(ax, [-gp.xp -gp.xp; gp.xp gp.xp].', [0 gp.hp; 0 gp.hp].', 'k', 'LineWidth', 2);
        hSp = plot(ax, NaN, NaN, 'k', 'LineWidth', 1.3); hDa = plot(ax, NaN, NaN, 'k', 'LineWidth', 1.3);
        % targets, start, trace
        hX = gobjects(1, size(r.tg, 2));
        for i = 1:size(r.tg, 2), hX(i) = plot(ax, r.tg(1, i), r.tg(2, i), 'x', 'MarkerSize', 14, 'LineWidth', 2.5); end
        plot(ax, r.x(1, 1), r.x(2, 1), 's', 'MarkerSize', 9, 'MarkerFaceColor', compC, 'MarkerEdgeColor', compC);
        hTr = plot(ax, NaN, NaN, 'Color', C, 'LineStyle', lsty.(g{1}), 'LineWidth', 1.8);
        % dancer
        hB = plot(ax, NaN, NaN, '-', 'Color', bodyC, 'LineWidth', 7);
        hH = rectangle(ax, 'Position', [0 0 2*p.rh 2*p.rh], 'Curvature', [1 1], 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', bodyC, 'LineWidth', 2.5);
        hJ = plot(ax, NaN, NaN, 'o', 'MarkerSize', 9, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', bodyC, 'LineWidth', 1.8);
        hE = plot(ax, NaN, NaN, '.', 'Color', bodyC, 'MarkerSize', 20);
        % CoM (plant), model CoM h(q_bar), reference
        if F.xbar, hXb = plot(ax, NaN, NaN, 'o', 'MarkerSize', 13, 'LineWidth', 2, 'Color', C); end
        hC = plot(ax, NaN, NaN, 'o', 'MarkerSize', 12, 'MarkerFaceColor', C, 'MarkerEdgeColor', 'k', 'LineWidth', 1);
        hRef = plot(ax, NaN, NaN, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', [1 0 0]);
        if F.xbar
            legend(ax, [hC hXb], {'CoM $x$ (plant)', 'model CoM $h(\bar q)$'}, 'Interpreter', 'latex', 'FontSize', 13, ...
                   'Location', 'northeast', 'AutoUpdate', 'off');
        end
        lt = lab.(g{1}); if ~isempty(F.note), lt = {lt, F.note}; end
        text(ax, XL(2) - 0.02*diff(XL), YL(1) + 0.02*diff(YL), lt, 'Interpreter', 'latex', 'FontSize', 17, ...
             'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'BackgroundColor', 'w', 'Margin', 2);
        hT = text(ax, XL(1) + 0.03*diff(XL), YL(2) - 0.03*diff(YL), '', 'Interpreter', 'latex', 'FontSize', 16, ...
             'VerticalAlignment', 'top', 'BackgroundColor', 'w');
        vw = VideoWriter(fullfile(odir, [g{1} '.mp4']), 'MPEG-4'); vw.FrameRate = FPS; vw.Quality = 95; open(vw);
        for j = 1:N
            Pj = r.P(:, :, j);
            if any(ghost == j)                               % leave a faded copy of this pose behind
                [bx, by] = body(Pj); h = plot(ax, bx, by, 'Color', ghostC, 'LineWidth', 4); uistack(h, 'bottom');
                h2 = rectangle(ax, 'Position', [Pj(:, 4).' - p.rh, 2*p.rh 2*p.rh], 'Curvature', [1 1], 'EdgeColor', [0.8 0.8 0.8], ...
                               'LineWidth', 1.2); uistack(h2, 'bottom');
                uistack(findobj(ax, 'Type', 'patch'), 'bottom');
            end
            [bx, by] = body(Pj); set(hB, 'XData', bx, 'YData', by);
            set(hH, 'Position', [Pj(:, 4).' - p.rh, 2*p.rh 2*p.rh]);
            set(hJ, 'XData', Pj(1, 2:3), 'YData', Pj(2, 2:3)); set(hE, 'XData', Pj(1, [1 5 6]), 'YData', Pj(2, [1 5 6]));
            gp = glyph(r.Q(j, 4), p); set(hSp, 'XData', gp.sx, 'YData', gp.sy); set(hDa, 'XData', gp.dx, 'YData', gp.dy);
            set(hTr, 'XData', r.x(1, 1:j), 'YData', r.x(2, 1:j));
            set(hC, 'XData', r.x(1, j), 'YData', r.x(2, j));
            if F.xbar, set(hXb, 'XData', r.xb(1, j), 'YData', r.xb(2, j)); end
            for i = 1:numel(hX)                                  % active target bold red, the others faded
                if i == r.act(j), set(hX(i), 'Color', [1 0 0], 'MarkerSize', 15); else, set(hX(i), 'Color', [1 0.7 0.7], 'MarkerSize', 11); end
            end
            set(hRef, 'XData', r.xr(1, j), 'YData', r.xr(2, j));
            set(hT, 'String', sprintf('$t = %.1f$ s', r.t(j)));
            writeVideo(vw, frame(fig));
        end
        close(vw); close(fig);
        fprintf('%s/%s.mp4: %d frames (%.1f s)\n', F.name, g{1}, N, (N - 1)/FPS);
    end

    % ---- time panels (curves revealed progressively)
    r0 = R.(laws{1}); T = r0.t(end); N = numel(r0.t); if ~isnan(NMAX), N = min(N, NMAX); end; tb = (1:size(r0.tg, 2) - 1)*(T/size(r0.tg, 2));
    for d = F.tpanels
        if ~isempty(ONLY) && ~ismember(d{1}, ONLY), continue; end
        fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', 'Position', [100 100 960 480]); theme(fig, 'light');
        ax = axes(fig, 'Units', 'pixels', 'Position', [110 80 820 370]); hold(ax, 'on'); grid(ax, 'on');
        set(ax, 'XLim', [0 T], 'TickLabelInterpreter', 'latex', 'FontSize', 16, 'LineWidth', 1, 'Box', 'on', 'GridAlpha', 0.2, 'Color', 'w');
        xlabel(ax, 'Time $t$ [s]', 'Interpreter', 'latex', 'FontSize', 20);
        switch d{1}
            case {'xc', 'zc'}
                k = 1 + strcmp(d{1}, 'zc'); Yf = @(g) R.(g).x(k, :);
                Y = cell2mat(cellfun(Yf, laws, 'UniformOutput', false)); yl = [min([Y, r0.xr(k, :)]), max([Y, r0.xr(k, :)])];
                yl = yl + 0.08*diff(yl)*[-1 1]; set(ax, 'YLim', yl);
                ylabel(ax, sprintf('$%s_c(t)$ [m]', char('x' + 2*(k == 2))), 'Interpreter', 'latex', 'FontSize', 20);
                hG = plot(ax, r0.t, r0.xr(k, :), 'k:', 'LineWidth', 1.5); gl = 'Reference';
                loc = 'northeast'; if k == 2, loc = 'southeast'; end
            case 'err'
                Yf = @(g) max(R.(g).err, 1e-2); set(ax, 'YScale', 'log', 'YLim', [0.1 400]);
                ylabel(ax, '$\|x_{\rm ref}(t) - x(t)\|$ [mm]', 'Interpreter', 'latex', 'FontSize', 20);
                hG = plot(ax, [0 T], [5 5], 'k:', 'LineWidth', 1.5); gl = '5 mm'; loc = 'southwest';
        end
        yl = get(ax, 'YLim'); for u = tb, plot(ax, [u u], yl, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); end
        H = gobjects(1, numel(laws));
        for i = 1:numel(laws), H(i) = plot(ax, NaN, NaN, 'Color', col.(laws{i}), 'LineStyle', lsty.(laws{i}), 'LineWidth', 1.8); end
        legend(ax, [hG, H], [{gl}, cellfun(@(g) lab.(g), laws, 'UniformOutput', false)], ...
               'Interpreter', 'latex', 'Location', loc, 'FontSize', 13, 'AutoUpdate', 'off');
        vw = VideoWriter(fullfile(odir, [d{1} '_t.mp4']), 'MPEG-4'); vw.FrameRate = FPS; vw.Quality = 95; open(vw);
        for j = 1:N
            for i = 1:numel(laws), y = Yf(laws{i}); set(H(i), 'XData', R.(laws{i}).t(1:j), 'YData', y(1:j)); end
            writeVideo(vw, frame(fig));
        end
        close(vw); close(fig);
        fprintf('%s/%s_t.mp4: %d frames (%.1f s)\n', F.name, d{1}, N, (N - 1)/FPS);
    end
end
disp('DONE make_videos_ballerina')
end

function P = pts(Q, p)
% Drawn points (2 x 6 x N) for N configurations Q (N x 4): pointe, hip, shoulder, head centre,
% working foot, hand (same as model/bal_points, vectorized).
e = @(th) [sin(th); cos(th)];
qa1 = Q(:, 1).'; qa2 = Q(:, 2).'; qa3 = Q(:, 3).'; qu = Q(:, 4).'; N = numel(qu);
P = zeros(2, 6, N);
P(:, 2, :) = p.Ls*e(qu);
P(:, 3, :) = squeeze(P(:, 2, :)) + p.Lt*e(qa1 + qu);
P(:, 4, :) = squeeze(P(:, 2, :)) + (p.Lt + p.rh)*e(qa1 + qu);
P(:, 5, :) = squeeze(P(:, 2, :)) - p.Lw*e(qa2 + qu);
P(:, 6, :) = squeeze(P(:, 3, :)) - p.La*e(qa1 + qa3 + qu);
end

function [bx, by] = body(P)
% Rods as one NaN-separated polyline: stance leg, torso, working leg, arm.
I = [1 2 NaN 2 3 NaN 2 5 NaN 3 6];
bx = NaN(size(I)); by = bx; k = ~isnan(I); bx(k) = P(1, I(k)); by(k) = P(2, I(k));
end

function gp = glyph(qu, p) %#ok<INUSD>
% Pointe spring (left) and damper (right), drawn between two ground posts and a point of the
% stance leg at distance s from the pointe (symbolic glyph, not to scale).
gp.xp = 0.24; gp.hp = 0.22; s = 0.14; zA = 0.14;
B = s*[sin(qu); cos(qu)];
% spring: zig-zag from the left post to B
A = [-gp.xp; zA]; u = B - A; L = norm(u); u = u/L; n = [-u(2); u(1)]; m = 7; w = 0.025;
sl = [0, 0.15, 0.15 + (0.5:m - 0.5)/m*0.7, 0.85, 1]; ofs = [0, 0, w*(-1).^(1:m), 0, 0];
S = A + u*(sl*L) + n*ofs; gp.sx = S(1, :); gp.sy = S(2, :);
% damper: rod, cup (open towards B), piston plate, rod to B
A = [gp.xp; zA]; u = B - A; L = norm(u); u = u/L; n = [-u(2); u(1)]; w = 0.03;
c0 = A + 0.30*L*u; c1 = A + 0.70*L*u; pp = A + 0.55*L*u;
D = [A, c0, NaN(2, 1), c1 + w*n, c0 + w*n, c0 - w*n, c1 - w*n, NaN(2, 1), pp + 0.75*w*n, pp - 0.75*w*n, NaN(2, 1), pp, B];
gp.dx = D(1, :); gp.dy = D(2, :);
end

function im = frame(fig)
% Off-screen frame grab at 96 dpi, cropped to even dimensions (required by the MPEG-4 encoder).
im = print(fig, '-RGBImage', '-r96');
im = im(1:2*floor(size(im, 1)/2), 1:2*floor(size(im, 2)/2), :);
end

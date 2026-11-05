function C = pcc_cases(name)
%PCC_CASES  The simulation cases of the release (and only these).
%
%   C = pcc_cases()      struct array with all cases
%   C = pcc_cases(name)  the case called name (error if unknown)
%
%   Case names: <robot>[_tri]_<law>[_first | _second]
%     robot   stiff : base stiffness (kscale = 1, "k200100"), 5 s per target, dt = 1e-5
%             soft  : kscale = 0.03 ("k63"), 20 s per target, dt = 1e-5 (single target) or
%                     3e-5 (triangle)
%     _tri    triangle (0.6, 0.6) -> (0.6, 0.4) -> (0.75, 0.5), each target held for the time
%             above; the reference slides linearly from one target to the next during the first
%             half of each hold (tramp = hold/2). Without _tri: single target (0.6, 0.6).
%     law     a1ff, a1fb (Alg. 1, feedforward / feedback), a2ff, a2fb (Alg. 2)
%     _first, _second   Alg. 1 with the static branch to 1st / 2nd order in gravity
%                       (clik_alg1.m); without suffix Alg. 1 neglects gravity in the statics
%
%     stiff_a1ff  stiff_a1fb  stiff_a2ff  stiff_a2fb              Fig. 18
%     soft_a1ff   soft_a1fb   soft_a2ff   soft_a2fb               Fig. 19
%     soft_a1ff_first   soft_a1fb_first   soft_a1ff_second   soft_a1fb_second
%     stiff_tri_a1ff  stiff_tri_a1fb  stiff_tri_a2ff  stiff_tri_a2fb
%     soft_tri_a1ff   soft_tri_a1fb   soft_tri_a2ff   soft_tri_a2fb
%     soft_tri_a1ff_first   soft_tri_a1fb_first   soft_tri_a1ff_second   soft_tri_a1fb_second
%
%   Each element: name, o (options for pcc_sim), group (figure / video set it belongs to).
laws4 = {'a1ff', 'a1fb', 'a2ff', 'a2fb'};
laws_g = {'a1ff_first', 'a1fb_first', 'a1ff_second', 'a1fb_second'};
C = struct('name', {}, 'o', {}, 'group', {});
C = add(C, 'stiff', laws4, false, 'fig18');
C = add(C, 'soft', laws4, false, 'fig19');
C = add(C, 'soft', laws_g, false, 'soft_gravity_order');
C = add(C, 'stiff', laws4, true, 'stiff_triangle');
C = add(C, 'soft', [laws4, laws_g], true, 'soft_triangle');
if nargin > 0
    i = find(strcmp({C.name}, name), 1);
    if isempty(i)
        error('pcc_cases: unknown case ''%s''. Known cases:\n  %s', name, strjoin({C.name}, '\n  '));
    end
    C = C(i);
end
end

function C = add(C, robot, laws, tri, group)
for i = 1:numel(laws)
    law = laws{i};
    o = struct('alg', str2double(law(2)), 'mode', law(3:4), 'statics', 'zero');
    if contains(law, '_first'),  o.statics = 'first';  end
    if contains(law, '_second'), o.statics = 'second'; end
    if strcmp(robot, 'soft'), o.kscale = 0.03; hold_time = 20; else, o.kscale = 1; hold_time = 5; end
    o.dt = 1e-5;
    name = robot;
    if tri
        o.targets = [0.6 0.6 0.75;                    % triangle, one target per column
                     0.6 0.4 0.5];
        o.tswitch = hold_time*(0:2);
        o.tramp = 0.5*hold_time;
        o.T = 3*hold_time;
        if strcmp(robot, 'soft'), o.dt = 3e-5; end
        name = [name '_tri'];
    else
        o.targets = [0.6; 0.6];
        o.tswitch = 0;
        o.tramp = 0;
        o.T = hold_time;
    end
    C(end + 1) = struct('name', [name '_' law], 'o', o, 'group', group); %#ok<AGROW>
end
end

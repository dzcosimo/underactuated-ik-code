function o = bal_scenario(name)
%BAL_SCENARIO  Options struct (input of bal_sim) of a named run.
%
%   o = bal_scenario(name),   name = <set>_<law>
%     set  'main'     nominal: plant and CLIK model with k = 200
%          'mismatch' plant k = 200, CLIK model and low level with 0.8 k = 160
%          'soft'     plant and model with k_soft = 120, start at the stable T1 equilibrium,
%                     slide to T2 (an unstable equilibrium at this stiffness); T3 not used
%     law  'a1ff' | 'a1fb' | 'a2ff' | 'a2fb'
%   Released runs: main_a1ff, main_a1fb, main_a2ff, main_a2fb, mismatch_a2ff, mismatch_a2fb,
%   soft_a2ff. Start and targets were chosen by kinematic screening (foot clearance, arm
%   elevation, sigma_min(J), H_uu margin); see README.md, section "Scenarios".

p = bal_params();
o.qa0 = [0.6; 1.15; -2.65];             % start: torso 40 deg forward, working leg back, arm raised forward-up
o.qu0 = 0;                              % Newton warm start for phi(qa0)
o.targets = [0.35  0.00 -0.35;          % T1 arabesque penchee (CoM forward, lowered)
             0.86  1.00  0.97];         % T2 centred, raised (arabesque, arm up); T3 cambre to the other side
o.hold = 6; o.ramp = 3; o.T = 18; o.dt = 1e-3; o.dtlog = 1e-3;
o.Gp = 3; o.dqu_bar0 = 0.15;            % statics gain Gamma_perp; Alg. 2 initial offset of qu_bar
o.k_plant = p.k; o.k_model = p.k;
parts = split(name, '_');
if numel(parts) ~= 2, error('unknown scenario %s', name); end
o.law = parts{2}; o.name = name;
if ~ismember(o.law, {'a1ff', 'a1fb', 'a2ff', 'a2fb'}), error('unknown law in %s', name); end
% task gain Gamma: ff 3.0, fb 1.0 (fb is limited by the slow pointe sway, see README)
if strcmp(o.law(3:4), 'ff'), o.Gam = 3.0; else, o.Gam = 1.0; end
switch parts{1}
    case 'main'
    case 'mismatch', o.k_model = 0.8*p.k;
    case 'soft'
        % At k_soft the main start is unstable (M g z_c = 121.9 > 120). Start at rest AT the
        % stable T1 equilibrium of k_soft (H_uu = 8.6), then slide to T2 (model H_uu = -9.5).
        % qa0 comes from a continuation k = 200 -> 120 of the Alg. 1 flow at T1 (README).
        o.k_plant = p.ksoft; o.k_model = p.ksoft;
        o.qa0 = [0.12006; 0.43390; -1.84770]; o.qu0 = 0.38;
        o.targets = o.targets(:, 1:2); o.T = 15;
    otherwise, error('unknown scenario %s', name);
end
end

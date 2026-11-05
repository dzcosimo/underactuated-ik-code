function p = bal_params()
%BAL_PARAMS  Physical and control parameters of the elastic-pointe ballerina (SI units).
%
%   p = bal_params()   returns a struct with the body lengths/masses, gravity, the pointe
%   spring/damper and the gains of the low-level joint controller.
%
%   Configuration q = (qa1, qa2, qa3, qu), all angles positive clockwise, i.e. a segment at
%   absolute angle th points along e(th) = [sin th; cos th] (z up):
%     qa1  torso relative to the stance leg (hip joint, actuated)
%     qa2  working leg relative to the stance leg (hip joint, actuated); legs together at qa2 = 0
%     qa3  arm relative to the torso (shoulder, actuated); arm along the torso at qa3 = 0
%     qu   absolute angle of the stance leg at the pointe, from vertical (passive: spring k, damper b)
%   Bodies are uniform rods; the head is a point mass at the centre of the drawn head circle.
%   Used by model/gen_symbolic.m (geometry, masses) and by the simulation (k, b, P, D).
%   See README.md, section "System".

p.Ls = 0.9;  p.ms = 3;     % stance leg (pointe -> hip)
p.Lt = 0.6;  p.mt = 5;     % torso (hip -> shoulder)
p.Lw = 0.9;  p.mw = 3;     % working leg (hip -> foot)
p.La = 0.55; p.ma = 1.2;   % arm (shoulder -> hand)
p.rh = 0.1;  p.mh = 1.0;   % head: circle of radius rh on top of the torso, point mass mh at its centre
p.g  = 9.81;
p.M  = p.ms + p.mt + p.mw + p.ma + p.mh;   % total mass, 13.2 kg

% Passive pointe. Stability of the unactuated statics needs H_uu = k - M g z_c > 0
% (z_c = CoM height); with k = 200, k/(M g z_c) >= 1.5 on every pose the main runs visit.
p.k  = 200;                % pointe torsional stiffness [Nm/rad]
p.ksoft = 120;             % "soft" pointe (soft_* scenario): T1 stable (H_uu = 8.6), T2 has H_uu = -9.5
p.b  = 50;                 % pointe damper [Nms/rad] (damping ratio of the sway ~0.8)

% Low-level controller tau = G_a(q_bar) + P (qa_bar - qa) - D qa'
p.P  = 300*eye(3);         % joint stiffness [Nm/rad]
p.D  = 30*eye(3);          % joint damping [Nms/rad]
end

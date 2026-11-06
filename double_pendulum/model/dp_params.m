function p = dp_params(varargin)
%DP_PARAMS  Physical parameters of the tendon-driven double pendulum (paper Sec. 2.3).
%
%   p = dp_params()                 default values (l = m = 1, d = 10, g = 9.81, k = 0,
%                                   theta = 0, alpha = pi/2)
%   p = dp_params('k2', 10, 'theta', pi/8, ...)   override any field by name
%
%   Fields of p:
%     l1, l2   link lengths [m]           m1, m2   tip masses [kg]
%     d1, d2   joint damping (D = diag(d1,d2))
%     k1, k2   parallel spring stiffnesses (Eq. (6))      g  gravity acceleration
%     theta    actuation direction: A = [r1; r2] = [cos(theta); sin(theta)] (Eq. (5), r = 1)
%              (called beta in the original code)
%     alpha    angle defining the unactuated coordinate qu (Sec. 2.5.1)
%     r1, r2   pulley radii, derived from theta
%     q0, qd0  initial configuration / velocity of the robot (used by the dynamic simulation)

p = struct('l1', 1, 'l2', 1, 'm1', 1, 'm2', 1, 'g', 9.81, 'k1', 0, 'k2', 0, ...
           'd1', 10, 'd2', 10, 'theta', 0, 'alpha', pi/2, 'q0', [0; pi/2], 'qd0', [0; 0]);
for i = 1:2:numel(varargin)
    assert(isfield(p, varargin{i}), 'dp_params: unknown parameter "%s"', varargin{i});
    p.(varargin{i}) = varargin{i+1};
end
p.r1 = cos(p.theta);   % pulley radii in polar form, ||A|| = 1 (Eq. (5))
p.r2 = sin(p.theta);
p.q0 = p.q0(:); p.qd0 = p.qd0(:);
end

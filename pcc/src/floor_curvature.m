function q = floor_curvature(q)
%FLOOR_CURVATURE  Curvature floor used by the original code: every |q_i| < 1e-2 is set to +1e-2.
%   The generated PCC functions (M, G, H, ...) are of the form 0/0 at zero curvature; the floor
%   keeps them finite. Applied in the dynamics, the low-level controller, Alg. 2 and the
%   gravity-expanded statics of Alg. 1, always in body coordinates q.
q(abs(q) < 1e-2) = 1e-2;
end

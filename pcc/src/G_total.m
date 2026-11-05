function G = G_total(q, mdl)
%G_TOTAL  Potential (gravity + elastic) generalized force G(q) in body coordinates, for the
%   stiffness of mdl (see pcc_model.m): G = G_fcn(q) + (kscale - 1) K0 q.
%   q: [n x 1] curvatures; G: [n x 1].
G = G_fcn(q);
if mdl.kscale ~= 1, G = G + mdl.dK*q(:); end
end

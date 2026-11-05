function rt = CC_rt_planar(s, q)
%CC_RT_PLANAR  Homogeneous transform (3 x 3) from the base to the point at normalized arc
%   length s in [0, 1] of a planar constant-curvature segment of UNIT length and total
%   curvature (bending angle) q. Multiply rt(1:2, 3) by the segment length L for a generic
%   segment. Works with numeric and symbolic q (symbolic: used by model/generate_pcc_model.m).
%   The exact expression is 0/0 at q = 0: for numeric |q| < eps the straight segment is returned.
singularity = isnumeric(q) && abs(q) < eps;
if singularity
    rt = [1, 0, 0;
          0, 1, s;
          0, 0, 1];
else
    rt = [ cos(q*s),  sin(q*s), (1 - cos(q*s))/q;
          -sin(q*s),  cos(q*s),      sin(q*s) /q;
                  0,         0,               1];
end
end

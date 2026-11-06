function A = dp_actuation(p)
%DP_ACTUATION  Constant actuation matrix A = [r1; r2] of the tendon, Eq. (5).
A = [p.r1; p.r2];
end

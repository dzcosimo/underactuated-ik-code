function PZ = projector_PZ(T)
%PROJECTOR_PZ  Matrix P_Z of Eq. (43): lifts a statics correction in qu to a velocity
%   (qa_dot, qu_dot) that keeps the task h constant to 1st order:
%       P_Z = [-Ja^-1 Ju; I] (I - Huu^-1 Hua (-Ja^-1 Ju))^-1
%   T : struct from DP_ACT_TERMS evaluated at the CLIK state.
%   (Original name: calculateP_perpG_u.)
Ja_inv  = regularized_inverse(T.Ja);
Huu_inv = regularized_inverse(T.Huu);
I = eye(size(T.Ju, 2));
term3 = -Ja_inv*T.Ju;
PZ = [term3; I]*inv(I - Huu_inv*T.Hua*term3); %#ok<MINV>
end

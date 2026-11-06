function PE = projector_PE(T)
%PROJECTOR_PE  Matrix P_E of Eq. (43): lifts a task correction in qa to a velocity
%   (qa_dot, qu_dot) tangent to the equilibrium foliation (keeps Gu constant to 1st order):
%       P_E = [I; -Huu^-1 Hua] (I - Ja^-1 Ju (-Huu^-1 Hua))^-1
%   T : struct from DP_ACT_TERMS evaluated at the CLIK state.
%   (Original name: calculateP_parallelG_u.)
Ja_inv  = regularized_inverse(T.Ja);
Huu_inv = regularized_inverse(T.Huu);
I = eye(size(T.Ja));
term2 = -Huu_inv*T.Hua;
PE = [I; term2]*inv(I - Ja_inv*T.Ju*term2); %#ok<MINV>
end

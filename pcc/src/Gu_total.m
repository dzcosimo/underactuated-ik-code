function Gu = Gu_total(qa, qu, mdl)
%GU_TOTAL  Unactuated block G_u(qa, qu) of the potential force in actuation coordinates, for
%   the stiffness of mdl (see pcc_model.m). Statics of the unactuated coordinates: G_u = 0.
%   qa: [m x 1], qu: [(n-m) x 1]; Gu: [(n-m) x 1].
Gu = GuFunction_act(qa, qu);
if mdl.kscale ~= 1, Gu = Gu + mdl.dKa_u*[qa(:); qu(:)]; end
end

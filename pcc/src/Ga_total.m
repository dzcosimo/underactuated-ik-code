function Ga = Ga_total(qa, qu, mdl)
%GA_TOTAL  Actuated block G_a(qa, qu) of the potential force in actuation coordinates
%   (G_act = T' G(T [qa; qu])), for the stiffness of mdl (see pcc_model.m).
%   qa: [m x 1], qu: [(n-m) x 1]; Ga: [m x 1].
Ga = GaFunction_act(qa, qu);
if mdl.kscale ~= 1, Ga = Ga + mdl.dKa_a*[qa(:); qu(:)]; end
end

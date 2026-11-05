function H = Hua_total(qa, qu, mdl)
%HUA_TOTAL  H_ua = dG_u/dqa [(n-m) x m] in actuation coordinates, for the stiffness of mdl.
H = HuaFunction(qa, qu);
if mdl.kscale ~= 1, H = H + mdl.dKa_ua; end
end

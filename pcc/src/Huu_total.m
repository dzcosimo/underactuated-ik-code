function H = Huu_total(qa, qu, mdl)
%HUU_TOTAL  H_uu = dG_u/dqu [(n-m) x (n-m)] in actuation coordinates, for the stiffness of mdl.
H = HuuFunction(qa, qu);
if mdl.kscale ~= 1, H = H + mdl.dKa_uu; end
end

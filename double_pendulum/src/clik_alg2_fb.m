function qbar_act_dot = clik_alg2_fb(qbar_act, xbar, q_act_m, x_m, p, GammaZ, GammaE, variant)
%CLIK_ALG2_FB  Algorithm 2 used as a high-level feedback loop (Sec. 5.4, Fig. 14, Eq. (51)).
%
%   qbar_act : CLIK state [qbar_a; qbar_u] (reference for the low-level controller)
%   xbar     : task reference
%   q_act_m  : measured robot configuration in actuation coordinates [qa; qu]
%   x_m      : measured task (end-effector horizontal position of the robot)
%   GammaZ, GammaE : gains of the task / statics terms (0.5 and 1 in Figs. 15-16)
%   variant  : 'FF'      feedforward, Eq. (42): task error xbar - h(qbar), Ja at qbar
%              'FB task' task error xbar - x_m (blue path of Fig. 14), Ja at qbar
%              'FB all'  task error xbar - x_m and Ja evaluated at the measured q
%                        (blue + orange paths of Fig. 14)
%   In all variants, as in the code that produced Figs. 15-16, P_E, P_Z, Huu and Gu are
%   evaluated at the CLIK state qbar (Eq. (51) writes P_E at the measured q; the code used qbar).
%   (Original name: IK_a2_fb; fb_x_type / fb_q_b1_type selected the variant.)

Tbar = dp_act_terms(qbar_act(1), qbar_act(2), p);
switch variant
    case 'FF',      x_ik = Tbar.x;  Tja = Tbar;
    case 'FB task', x_ik = x_m;     Tja = Tbar;
    case 'FB all',  x_ik = x_m;     Tja = dp_act_terms(q_act_m(1), q_act_m(2), p);
    otherwise, error('clik_alg2_fb: unknown variant "%s"', variant);
end
block_1 =  projector_PE(Tbar)*regularized_inverse(Tja.Ja)*GammaZ*(xbar - x_ik);   % task term
block_2 = -projector_PZ(Tbar)*regularized_inverse(Tbar.Huu)*GammaE*Tbar.Gu;       % statics term
qbar_act_dot = block_1 + block_2;
end

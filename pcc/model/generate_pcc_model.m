function generate_pcc_model(outdir)
%GENERATE_PCC_MODEL  Symbolic derivation of the planar PCC robot model and export of the
%   MATLAB functions in model/generated/ (requires the Symbolic Math Toolbox).
%
%   generate_pcc_model()          writes into model/generated/ (overwrites the shipped files)
%   generate_pcc_model(outdir)    writes into outdir
%
%   Only needed if the physical parameters in src/pcc_params.m are changed (the stiffness can be
%   scaled at run time without regeneration, see src/pcc_model.m). The run can take a long time
%   (the simplification of the Coriolis matrix dominates).
%
%   Model (n CC segments, curvatures q, m = 2 actuated sections):
%     M(q) q_ddot + C(q, q_dot) q_dot + G(q) + D q_dot = A tau
%   - M from p.mpts point masses per segment (translational inertia only);
%   - C from the Christoffel symbols of M;
%   - G = gravity force of the mass distributed uniformly along each segment + K (q - q_rest);
%   - D = diag(b); A = [1 on the segments of section 1, 0 elsewhere | 1 on section 2, ...].
%   Actuation coordinates [qa; qu] = B q, B = [A'; N'] with N a basis of null(A') (constant,
%   so qa = A' q are the actuated coordinates), q = T [qa; qu], T = B^-1.
%
%   Exported functions (all used by the simulation):
%     M_fcn(q), C_fcn(q, dq), D_fcn(q), A_fcn(q), G_fcn(q)          dynamics (body coordinates)
%     q_to_q_act(q), q_act_to_q(qa, qu), Jacobiab_q_act_to_q(qa, qu) coordinate maps (T constant)
%     GaFunction_act, GuFunction_act (qa, qu)    G_act = T' G(T [qa; qu]) = [G_a; G_u]
%     HuaFunction, HuuFunction (qa, qu)          blocks dG_u/dqa, dG_u/dqu of T' H_G T
%     JaFunction, JuFunction (qa, qu)            task Jacobian blocks dh/dqa, dh/dqu (tip position)
%     qBarU(qa), J_qBarU(qa)                     zero-gravity static branch qu = qBarU(qa)
%                                                (elastic only, linear in qa) and its Jacobian
%   The symbolic functions take the parameter values of pcc_params at generation time; the
%   shipped files were generated with the base stiffness "k200100".
here = fileparts(mfilename('fullpath'));
if nargin < 1, outdir = fullfile(here, 'generated'); end
if ~exist(outdir, 'dir'), mkdir(outdir); end
addpath(fullfile(fileparts(here), 'src'));
out = @(name) fullfile(outdir, name);

p = pcc_params();
n_segments = p.n;
L = p.L;

%% Symbolic variables
q  = sym('q', [n_segments, 1], 'real');
dq = sym('dq', [n_segments, 1], 'real');
s  = sym('s', 'real');                          % normalized arc length

%% Actuation matrix A (constant): tau_1 acts on section 1 (first ceil(n/2) segments), tau_2 on
% section 2. "+ q*0" forces a dependency on q so that the generated function accepts q.
A_matrix = sym(zeros(n_segments, 2));
A_matrix(:, 1) = [ones(ceil(n_segments/2), 1); zeros(floor(n_segments/2), 1)];
A_matrix(:, 2) = [zeros(ceil(n_segments/2), 1); ones(floor(n_segments/2), 1)];
A_matrix = A_matrix + q*0;
matlabFunction(A_matrix, 'File', out('A_fcn'), 'Vars', {q}, 'Optimize', false);

%% Actuation coordinates (constant transform, valid because A is constant)
A0 = double(subs(A_matrix, q, zeros(n_segments, 1)));
[nA, mA] = size(A0);
assert(nA == n_segments && mA < n_segments && rank(A0) == mA);
N0 = null(A0.', 'r');                          % columns span null(A0')
B0 = [A0.'; N0.'];                             % [qa; qu] = B0 q
T0 = inv(B0);                                  % q = T0 [qa; qu]
B_sym = sym(B0);
T_sym = sym(T0);
qa = sym('qa', [mA, 1], 'real');
qu = sym('qu', [n_segments - mA, 1], 'real');
q_to_act_sym = B_sym*q;                       % [qa; qu] = B q
q_from_act_sym = T_sym*[qa; qu];              % q = T [qa; qu]
matlabFunction(q_to_act_sym, 'File', out('q_to_q_act'), 'Vars', {q}, 'Optimize', false);
matlabFunction(q_from_act_sym, 'File', out('q_act_to_q'), 'Vars', {qa, qu}, 'Optimize', false);
T_dep = T_sym + [qa; qu]*0;                    % dq/d[qa; qu] = T (constant)
matlabFunction(T_dep, 'File', out('Jacobiab_q_act_to_q'), 'Vars', {qa, qu}, 'Optimize', false);

%% Mass matrix M(q): p.mpts point masses per segment
M = sym(zeros(n_segments));
for i_M = 1:n_segments
    for i_m = 1:p.mpts
        x = forward_kinematics_planar_PCC(q(1:i_M), i_m/p.mpts, L);
        J = sym(zeros(3, n_segments));
        J(1, :) = gradient(x(1), q);
        J(2, :) = gradient(x(2), q);
        J(3, 1:i_M) = 1;                       % planar angular velocity
        m_point = p.mass(i_M)/p.mpts;
        M = M + simplify(J'*diag([m_point, m_point, 0])*J);
    end
end
matlabFunction(M, 'File', out('M_fcn'), 'Vars', {q});

%% Coriolis matrix C(q, dq) (Christoffel symbols of the first kind)
C = sym(zeros(n_segments));
for i_q = 1:n_segments
    for j_q = 1:n_segments
        for k_q = 1:n_segments
            Christoffel = 0.5*(diff(M(i_q, j_q), q(k_q)) + diff(M(i_q, k_q), q(j_q)) - diff(M(j_q, k_q), q(i_q)));
            C(i_q, j_q) = C(i_q, j_q) + Christoffel*dq(k_q);
        end
    end
end
C = simplify(C);
matlabFunction(C, 'File', out('C_fcn'), 'Vars', {q, dq}, 'Optimize', false);

%% Damping D (constant viscous)
D = diag(p.b) + q*0;
matlabFunction(D, 'File', out('D_fcn'), 'Vars', {q}, 'Optimize', false);

%% Potential force G(q) = gravity (mass distributed along each segment) + K (q - q_rest)
G = sym(zeros(n_segments, 1));
grav_vec = [p.g*cos(p.phi_g); p.g*sin(p.phi_g); 0];
for i_G = 1:n_segments
    x = forward_kinematics_planar_PCC(q(1:i_G), s, L);
    J = sym(zeros(3, n_segments));
    J(1, :) = jacobian(x(1), q);
    J(2, :) = jacobian(x(2), q);
    J(3, 1:i_G) = 1;
    G = G + p.mass(i_G)*simplify(int(J'*grav_vec, s, 0, 1));
end
Kmat = diag(p.k);
G = simplify(G + Kmat*(q - p.q_rest));
H_G = jacobian(G, q);                          % = H_G_grav + K
matlabFunction(G, 'File', out('G_fcn'), 'Vars', {q}, 'Optimize', false);

%% G and its Hessian blocks in actuation coordinates: G_act = T' G(T [qa; qu]), H_act = T' H_G T
q_from_act = T_sym*[qa; qu];
G_act_sym = simplify(T_sym.'*subs(G, q, q_from_act));
Ga_sym = G_act_sym(1:mA, :);
Gu_sym = G_act_sym(mA+1:end, :);
matlabFunction(Ga_sym, 'File', out('GaFunction_act'), 'Vars', {qa, qu}, 'Optimize', false);
matlabFunction(Gu_sym, 'File', out('GuFunction_act'), 'Vars', {qa, qu}, 'Optimize', false);
H_act = simplify(T_sym.'*subs(H_G, q, q_from_act)*T_sym);
Hua_sym = H_act(mA+1:end, 1:mA);
Huu_sym = H_act(mA+1:end, mA+1:end);
matlabFunction(Hua_sym, 'File', out('HuaFunction'), 'Vars', {qa, qu}, 'Optimize', false);
matlabFunction(Huu_sym, 'File', out('HuuFunction'), 'Vars', {qa, qu}, 'Optimize', false);

%% Task Jacobian blocks (tip position) in actuation coordinates
x_ac = subs(forward_kinematics_planar_PCC(q, 1, L), q, q_from_act);
J_all = simplify(jacobian(x_ac, [qa; qu]));
Ja_sym = J_all(:, 1:mA);
Ju_sym = J_all(:, mA+1:end);
matlabFunction(Ja_sym, 'File', out('JaFunction'), 'Vars', {qa, qu}, 'Optimize', false);
matlabFunction(Ju_sym, 'File', out('JuFunction'), 'Vars', {qa, qu}, 'Optimize', false);

%% Zero-gravity static branch qu = qBarU(qa) = P qa + qu0 (elastic statics G_u = 0 without
% gravity): Kt = T' K T, b = T' K q_rest, P = -Kuu^-1 Kua, qu0 = Kuu^-1 b_u
Kt  = T0.'*double(Kmat)*T0;
b   = T0.'*double(Kmat)*double(p.q_rest);
Kua = Kt(mA+1:end, 1:mA);
Kuu = Kt(mA+1:end, mA+1:end);
b_u = b(mA+1:end);
[Vk, Dk] = eig((Kuu + Kuu.')/2);               % eigenvalue floor (inactive for this robot)
Kuu_reg = Vk*diag(max(diag(Dk), 1e-8))*Vk';
P_num   = -(Kuu_reg \ Kua);
q_u0num =  (Kuu_reg \ b_u);
qa_sym = sym('qa', [mA, 1], 'real');
qu_sym = sym(P_num)*qa_sym + sym(q_u0num);
P_dep = sym(P_num) + sum(qa_sym)*0;            % force the dependency on qa
matlabFunction(qu_sym, 'File', out('qBarU'), 'Vars', {qa_sym}, 'Optimize', false);
matlabFunction(P_dep, 'File', out('J_qBarU'), 'Vars', {qa_sym}, 'Optimize', false);

fprintf('PCC model functions generated in %s\n', outdir);
end

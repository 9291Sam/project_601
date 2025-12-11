function eom_deriver()
% EOM_DERIVER
% Generates Dynamics, A, and B matrices for the 10-State Hybrid Model.

% 1. DEFINE VARIABLES
syms x y theta phi Omega 'real';
alpha = sym('alpha', 'real');
syms dotx doty dottheta dotphi dotOmega dotalpha 'real';
syms ddotx ddoty ddottheta ddotphi ddotOmega ddotalpha 'real';

% Hybrid Driver Variables
syms v dv 'real' 

% Dynamic DOFs
q_dyn   = [theta; phi; alpha];
dq_dyn  = [dottheta; dotphi; dotalpha];
ddq_dyn = [ddottheta; ddotphi; ddotalpha];

% Inputs (2 Inputs)
syms u_wheel u_roll 'real'
u = [u_wheel; u_roll];

% Parameters
syms r d m_w m_s I_wpitch I_wyawroll g 'real' 
local_wheel_inertias = [I_wyawroll 0 0; 0 I_wpitch 0; 0 0 I_wyawroll];

% 2. KINEMATICS & ENERGY
Rz_theta = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
Rx_phi   = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry_alpha = [cos(alpha) 0 sin(alpha); 0 1 0; -sin(alpha) 0 cos(alpha)];

w_yaw   = [0; 0; dottheta];
w_roll  = Rz_theta * [dotphi; 0; 0];
w_wheel_spin = Rz_theta * Rx_phi * [0; v/r; 0]; 
w_wheel_global = w_yaw + w_roll + w_wheel_spin;

% Kinetic Energy
R_wheel = Rz_theta * Rx_phi;
w_wheel_local = R_wheel.' * w_wheel_global;
T_wheel_rot = 0.5 * w_wheel_local.' * local_wheel_inertias * w_wheel_local;

% Seat Velocity
v_hub_global = [v*cos(theta); v*sin(theta); 0];
r_seat_rel = Rz_theta * Rx_phi * Ry_alpha * [0; 0; d];
dr_seat_dt = jacobian(r_seat_rel, q_dyn) * dq_dyn; 
v_seat_global = v_hub_global + dr_seat_dt;

T_seat_trans = 0.5 * m_s * (v_seat_global.' * v_seat_global);
T = T_seat_trans + T_wheel_rot;

% Potential Energy
h_seat = r + r_seat_rel(3);
V = m_s * g * h_seat;
L = simplify(T - V);

% 3. LAGRANGE SOLVER
Q_forces = [0; u_roll; -u_wheel]; 

Eqs = sym(zeros(3, 1));
for i = 1:3
    var = q_dyn(i);
    dvar = dq_dyn(i);
    dL_dvar = diff(L, dvar);
    dL_var  = diff(L, var);
    dt_dL_dvar = jacobian(dL_dvar, [q_dyn; dq_dyn]) * [dq_dyn; ddq_dyn] + ...
                 jacobian(dL_dvar, v) * dv;
    Eqs(i) = dt_dL_dvar - dL_var - Q_forces(i);
end

[M_sys, F_sys] = equationsToMatrix(Eqs == 0, ddq_dyn);
accel_sol = M_sys \ F_sys;

% Drivers
dv_sol = (u_wheel/r) / (m_w + m_s);
dOmega_sol = v / r;

% 4. STATE SPACE CONSTRUCTION
% z = [x y theta v phi dphi alpha dalpha dtheta Omega]
dx_val = v * cos(theta);
dy_val = v * sin(theta);

% The full nonlinear function f(x, u)
dz = [dx_val; dy_val; dottheta; dv_sol; dotphi; accel_sol(2); dotalpha; accel_sol(3); accel_sol(1); dOmega_sol];
dz = subs(dz, dv, dv_sol);

z_sym = [x; y; theta; v; phi; dotphi; alpha; dotalpha; dottheta; Omega];
u_sym = [u_wheel; u_roll];

% 5. LINEARIZATION (Calculate Jacobians)
A_sym = jacobian(dz, z_sym);
B_sym = jacobian(dz, u_sym);

fprintf('Generating Files...\n');
matlabFunction(dz, 'File', 'getDynamics_Advanced', ...
    'Vars', {z_sym, u_sym, r, d, m_w, m_s, I_wpitch, I_wyawroll, g});

% Generate the A matrix function
matlabFunction(A_sym, 'File', 'getA_Advanced', ...
    'Vars', {z_sym, u_sym, r, d, m_w, m_s, I_wpitch, I_wyawroll, g});

% Generate the B matrix function
matlabFunction(B_sym, 'File', 'getB_Advanced', ...
    'Vars', {z_sym, u_sym, r, d, m_w, m_s, I_wpitch, I_wyawroll, g});

fprintf('Done.\n');
end
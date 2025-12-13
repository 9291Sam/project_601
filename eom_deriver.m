% https://github.com/MatthewPeterKelly/Lagrange_Mechanics_Derivations
% https://github.com/MatthewPeterKelly/Lagrange_Mechanics_Derivations/blob/28680024833fe3dd00af9d1ec23644b6f3d4de87/springCartPole/EoM_Spring_Cart_Pole.m

syms x y theta phi 'real';
syms dotx doty dottheta dotphi 'real';
syms ddotx ddoty ddottheta ddotphi 'real';

q  = [x; y; theta; phi];
dq = [dotx; doty; dottheta; dotphi];
ddq = [ddotx; ddoty; ddottheta; ddotphi];

% u_x - torque around the x axis, affects phi
% u_y - torque around the y axis, affects theta
syms u_x u_y 'real';

state_vector = [q; dq];
input_vector = [u_x; u_y];

% r          - ball radius
% d          - distance between ball com & body com
% m_ball     - mass of the ball
% m_body     - mass of the body
% I_ball     - moment of inertia of the ball (spherically symmetrical)
% I_body_xy   - moment of inertia of the body around the x axis
% I_body_z   - moment of inertia of the body around the z axis
% g          - gravitational constant
syms r d m_ball m_body I_ball I_body_xy I_body_z g 'real';

% order of rotations
% theta -> phi

Rx_phi = [
    1  0         0;
    0  cos(phi) -sin(phi);
    0  sin(phi)  cos(phi)
];
rotate_by_phi = Rx_phi;

Ry_theta = [
    cos(theta)  0  sin(theta);
    0           1  0;
    -sin(theta) 0  cos(theta)
];
rotate_by_theta = Ry_theta;

R_body = Ry_theta * Rx_phi;

global_position_contact_point = [x; y; 0];
global_position_ball_center = global_position_contact_point + [0; 0; r];
global_position_body_com = global_position_ball_center + R_body * [0; 0; d]; % [0; 0; d] is the local vector from the ball to the body

% Linear Velocities
global_velocity_ball = jacobian(global_position_ball_center, q) * dq;
global_velocity_body_com = jacobian(global_position_body_com, q) * dq;

% Angular Velocities
local_angular_velocity_ball = [-doty/r; dotx/r; 0]; 
local_angular_velocity_body = R_body'*([0; dottheta; 0] + Ry_theta * [dotphi; 0; 0]);

% Linear energies | E = 1/2 m v^2
T_translational_ball = 0.5 * m_ball * (global_velocity_ball'*global_velocity_ball);
T_translational_body = 0.5 * m_body * (global_velocity_body_com'*global_velocity_body_com);

% Angular Energies | E = 1/2 I w^2 = 1/2 w^T I w
T_rotational_ball = 0.5 * I_ball * (local_angular_velocity_ball'*local_angular_velocity_ball);
T_rotational_body = 0.5 * (local_angular_velocity_body' * diag([I_body_xy, I_body_xy, I_body_z]) * local_angular_velocity_body);

% Potential Energies
U_ball = m_ball * global_position_ball_center(3) * g;
U_body = m_body * global_position_body_com(3) * g;


T = T_translational_ball + T_translational_body + T_rotational_ball + T_rotational_body;
U = U_ball + U_body;

L = simplify(T - U);

% past here is stolen from
% https://github.com/MatthewPeterKelly/Lagrange_Mechanics_Derivations/blob/28680024833fe3dd00af9d1ec23644b6f3d4de87/springCartPole/EoM_Spring_Cart_Pole.m
dL_dq  = jacobian(L, q)';
dL_ddq = jacobian(L, dq)';
d_dt_dL_ddq = jacobian(dL_ddq, [q; dq]) * [dq; ddq];

EoM = simplify(d_dt_dL_ddq - dL_dq); 

B = [
    0,   -1/r;
    1/r, 0;
    0,   1;
    1,   0
];

solutions = solve(EoM == B*input_vector, ddq);

state_updates = [
    solutions.ddotx;
    solutions.ddoty;
    solutions.ddottheta;
    solutions.ddotphi
];

constants = [r, d, m_ball, m_body, I_ball, I_body_xy, I_body_z, g];

variables = [
    x, y, theta, phi, ...          
    dotx, doty, dottheta, dotphi, ... 
    u_x, u_y, ...      
    constants...
];

matlabFunction(state_updates, "File", "ballbot_dynamics", "Vars", variables);

dstate_vector = [dq; state_updates];

A_symbolic = jacobian(dstate_vector, state_vector);
B_symbolic = jacobian(dstate_vector, input_vector);

linearization_variables = [x, y, theta, phi, dotx, doty, dottheta, dotphi, u_x, u_y];
linearization_values    = zeros(size(linearization_variables));

A = simplify(subs(A_symbolic, linearization_variables, linearization_values));
B = simplify(subs(B_symbolic, linearization_variables, linearization_values));

matlabFunction(A, "File", "get_A_matrix", "Vars", constants);
matlabFunction(B, "File", "get_B_matrix", "Vars", constants);

function eom_deriver()
% https://github.com/MatthewPeterKelly/Lagrange_Mechanics_Derivations
% https://github.com/MatthewPeterKelly/Lagrange_Mechanics_Derivations/blob/28680024833fe3dd00af9d1ec23644b6f3d4de87/springCartPole/EoM_Spring_Cart_Pole.m

syms x y theta phi Omega 'real';
alpha = sym('alpha', 'real');
syms dotx doty dottheta dotphi dotOmega dotalpha 'real';
syms ddotx ddoty ddottheta ddotphi ddotOmega ddotalpha 'real';

q  = [x; y; theta; phi; Omega; alpha];
dq = [dotx; doty; dottheta; dotphi; dotOmega; dotalpha];
ddq = [ddotx; ddoty; ddottheta; ddotphi; ddotOmega; ddotalpha];

% t_1 - wheel pitch
% t_2 - seat roll
% t_3 - seat pitch
syms t_1 t_2 t_3 'real';


% r          - wheel radius
% d          - distance between fork and hub
% m_w        - wheel mass
% m_s        - seat mass
% I_wpitch   - Wheel pitch moment
% I_wyawroll - Wheel roll moment
% g          - gravitational constant
syms r d m_w m_s I_wpitch I_wyawroll g 'real'; 

local_wheel_inertias = [I_wyawroll 0 0; 0 I_wpitch 0; 0 0 I_wyawroll];

% Order of rotations. 
% {alpha / omega} -> phi -> theta -> world

% these matrices bring things into global space from the local spaces
Rz_theta = [
    cos(theta) -sin(theta) 0; 
    sin(theta) cos(theta)  0;
    0          0           1
];

rotate_by_theta = Rz_theta;

Rx_phi = [
    1 0        0;
    0 cos(phi) -sin(phi);
    0 sin(phi) cos(phi);
];

rotate_by_phi = Rx_phi;

Ry_alpha = [
    cos(alpha)  0 sin(alpha);
    0           1 0;
    -sin(alpha) 0 cos(alpha);
];

rotate_by_alpha = Ry_alpha;

% global origin -> unicycle_base
root = [x; y; 0];

% vector between global origin -> wheel center
wheel_hub_global_pos = root + rotate_by_theta * rotate_by_phi * [0; 0; r];
% global origin -> seat center
seat_global_pos = wheel_hub_global_pos + rotate_by_theta * rotate_by_phi * rotate_by_alpha * [0; 0; d];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lagrangian shit

% time derivative & chain rule 
% jacobian(wheel_hub_global_pos, q): 
wheel_hub_global_velocity = jacobian(wheel_hub_global_pos, q) * dq;
seat_global_velocity      = jacobian(seat_global_pos, q) * dq;

% T = 0.5 m v^2
T_wheel_translation = 0.5 * m_w * (wheel_hub_global_velocity.'*wheel_hub_global_velocity);
T_seat_translation = 0.5 * m_s * (seat_global_velocity.'*seat_global_velocity);




% bring rotations into global space, then put them in the frame of the wheel
wheel_yaw_world = [0; 0; dottheta];
wheel_roll_world = Rz_theta * [dotphi; 0; 0];
wheel_pitch_world = Rz_theta * Rx_phi * [0; dotOmega; 0];
wheel_rotations_world = wheel_yaw_world + wheel_roll_world + wheel_pitch_world;
global_rotations_to_wheel_local_rotations = (rotate_by_theta * rotate_by_phi).';
wheel_rotations_local = global_rotations_to_wheel_local_rotations * wheel_rotations_world;

% 0.5 * w' J w
T_wheel_rotation = 0.5 * wheel_rotations_local.' * local_wheel_inertias * wheel_rotations_local;


U_wheel = m_w * g * wheel_hub_global_pos(3);
U_seat = m_s * g * seat_global_pos(3);



T = T_wheel_translation + T_seat_translation + T_wheel_rotation;
U = U_wheel + U_seat;
L = simplify(T - U);


% Everything below is stolen straight from 
% https://github.com/MatthewPeterKelly/Lagrange_Mechanics_Derivations/blob/28680024833fe3dd00af9d1ec23644b6f3d4de87/springCartPole/EoM_Spring_Cart_Pole.m
% Lagrangian
% ∂ = p
% (d/dt)(pL/pdq) - (pL/pq) = generalized forces

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  evaluate partial derivatives                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%              DL  
%  DL_Dq  ==  ---      Note that 'D' is partial derivative here 
%              Dq
%
DL_Dq = jacobian(L,q)';

%              DL  
%  DL_Ddq  ==  ---      Note that 'D' is partial derivative here 
%              Ddq
%
DL_Ddq = jacobian(L,dq)';

%                D  / DL  \         * Note that some of those 'd' should be
% DDL_DtDdq  ==  -- | --  |         curvy 'D' to represent partial
%                Dt \ Ddq /         derivatives
%
% Note the application of the chain rule:  (Quoting Andy Ruina: )
%      d BLAH / dt  =  jacobian(BLAH, [q qdot])*[qdot qddot]'
%
DDL_DtDdq = jacobian(DL_Ddq,[q; dq]) * [dq; ddq];


%Write out as single equation and simplify:
EoM = DDL_DtDdq - DL_Dq; % TODO: why was this flipped in the example??
EoM = simplify(EoM); 

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   mass matrix gymnastics                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% This is a confusing step - read carefully:
%
% We know that our equations are of the form:
%             EoM = M(q,dq)*qdd + f(q,dq) == 0;
%
% Thus, we can find M(q,dq) by using the jacobian command:

M = jacobian(EoM,ddq);

% Now, we want to find f(q,dq). We can do this by substituting in zero for
% the acceleration vector (dqq)

f = subs(EoM,ddq,sym([0; 0; 0; 0; 0; 0]));

% https://www.mathworks.com/help/symbolic/sym.matlabfunction.html
matlabFunction(M, "File", "getMassMatrix", "Vars", [q; dq; r; d; m_w; m_s; I_wpitch; I_wyawroll; g]);
matlabFunction(f, "File", "getFVector", "Vars", [q; dq; r; d; m_w; m_s; I_wpitch; I_wyawroll; g]);

% when simulating, n
% EoM = M(q,dq)*qdd + f(q,dq) == 0;
%  M(q,dq)*qdd == -f(q,dq);
% qdd = M(q, dq)^-1 * -f(q, dq)

% assuming input vector is a column vector [t_1; t_2; t_3]
B = [
    0 0 0;
    0 0 0;
    0 0 0;
    0 1 0;
    1 0 0;
    0 0 1;
];
matlabFunction(B, "File", "getInputMatrix",  "Vars", [q; dq; r; d; m_w; m_s; I_wpitch; I_wyawroll; g]);

% inputs
% EoM = M(q,dq)*qdd + f(q,dq) == 0;
% EoM = M(q,dq)*qdd + f(q,dq) == sum Q; (601 document) 
% each tau is a vector 
% Q = 

% t_1 - torque on the wheel (pitch)
% t_2 - torque on the seat (pitch)
% t_3 - torque on the seat (roll)
%https://farside.ph.utexas.edu/teaching/336k/Newton/node90.html

% fuck ass constraints
% dotx * sin(theta) - doty cos(theta) = 0
% dotx * cos(theta) + doty sin(theta) = r * dotOmega


%M(q, dq)ddq + f(q, dq) = Bu

% row vectors because constraint * dq == 0
lateral_constraint = [sin(theta), -cos(theta), 0, 0, 0, 0];
rolling_constraint = [cos(theta), sin(theta), 0, 0, -r, 0];

C = [
    lateral_constraint;
    rolling_constraint
];
% C * dq == 0
C_eq0 = C * dq;
time_derivative_c_eq0 = jacobian(C_eq0, [q; dq]) * [dq; ddq];
gamma = subs(time_derivative_c_eq0, ddq, [0; 0; 0; 0; 0; 0]);

matlabFunction(C, "File", "getConstraintMatrix", "Vars", [q; dq; r; d; m_w; m_s; I_wpitch; I_wyawroll; g]);
matlabFunction(gamma, "File", "getConstraintGamma", "Vars", [q; dq; r; d; m_w; m_s; I_wpitch; I_wyawroll; g]);

end



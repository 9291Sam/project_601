clc; clear; close all;

% T = 0.5 m v^2 -> T = 0.5 v.' m v

% states
syms x y theta phi Omega alpha;
syms dotx doty dottheta dotphi omega dotalpha;

q  = [x; y; theta; phi; Omega; alpha];
dq = [dotx; doty; dottheta; dotphi; omega; dotalpha];

% constants
% r            - wheel radius
% d            - distance between fork and hub
% m_w          - wheel mass
% m_s          - seat mass
% I_w{x, y, z} - Wheel moments
% g            - gravitational constant
syms r d m_w m_s I_wx I_wy I_wz g; 

local_wheel_inertias = [I_wx 0 0; 0 I_wy 0; 0 0 I_wz];
local_seat_inertias  = [m_s*d 0 0; 0 m_s*d 0; 0 0 0];


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

root = [x; y; 0];

% vector between root and wheel center rotated by theta & phi
wheel_hub_global_pos = root + rotate_by_theta * rotate_by_phi * [0; 0; r];
seat_global_pos = wheel_hub_global_pos + rotate_by_theta * rotate_by_phi * rotate_by_alpha * [0; 0; d];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lagrangian shit

wheel_hub_global_velocity = jacobian(wheel_hub_global_pos, q) * dq;
seat_global_velocity      = jacobian(seat_global_pos, q) * dq;

% T = 0.5 m v^2
T_wheel_translation = 0.5 * m_w * (wheel_hub_global_velocity.'*wheel_hub_global_velocity);
T_seat_translation = 0.5 * m_s * (seat_global_velocity.'*seat_global_velocity);




% bring rotations into global space, then put them in the frame of the wheel
wheel_yaw_world = [0; 0; dottheta];
wheel_roll_world = Rz_theta * [dotphi; 0; 0];
wheel_pitch_world = Rz_theta * Rx_phi * [0; omega; 0];
wheel_rotations_world = wheel_yaw_world + wheel_roll_world + wheel_pitch_world;
global_rotations_to_wheel_local_rotations = (rotate_by_theta * rotate_by_phi).';
wheel_rotations_local = global_rotations_to_wheel_local_rotations * wheel_rotations_world;

% 0.5 * w' J w
T_wheel_rotation = 0.5 * wheel_rotations_local.' * local_wheel_inertias * wheel_rotations_local;

% TODO: its already captured????
T_seat_rotation = 0; 


U_wheel = m_w * g * wheel_hub_global_pos(3);
U_seat = m_s * g * seat_global_pos(3);



% L = T - U
T_total = T_wheel_translation + T_seat_translation + T_wheel_rotation + T_seat_rotation;
U_total = U_wheel + U_seat;
L = simplify(T_total - U_total);
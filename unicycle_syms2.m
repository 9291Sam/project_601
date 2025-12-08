% syms theta phi alpha Omega 'real'
% syms dottheta dotphi dotalpha dotOmega 'real'
% syms ddottheta ddotphi ddotalpha ddotOmega 'real'

% % 1. NEW STATE VECTOR (No x or y!)
% q   = [theta; phi; alpha; Omega];
% dq  = [dottheta; dotphi; dotalpha; dotOmega];
% ddq = [ddottheta; ddotphi; ddotalpha; ddotOmega];

% syms r d m_w m_s I_wpitch I_wyawroll g 'real'; 

% % Rotation Matrices (Standard Z-X-Y Euler sequence)
% R_yaw   = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]; % Z
% R_roll  = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];         % X
% R_pitch = [cos(alpha) 0 sin(alpha); 0 1 0; -sin(alpha) 0 cos(alpha)]; % Y

% % 2. DEFINE VELOCITIES VIA CONSTRAINTS
% % The magic happens here. We don't define 'pos', we define 'vel' directly.

% % Velocity of the Wheel Hub
% % Magnitude is radius * wheel_speed
% % Direction is determined by the Yaw (theta)
% speed_hub = r * dotOmega;
% vel_hub_world = R_yaw * [speed_hub; 0; 0]; 

% % Angular Velocity of the Seat Frame (Fork)
% % Sum of angular rates transformed into world frame
% w_frame_world = [0;0;dottheta] + ...              % Yaw
%                 R_yaw*[dotphi;0;0] + ...          % Roll
%                 R_yaw*R_roll*[0;dotalpha;0];      % Pitch

% % Velocity of the Seat (Point Mass)
% % v_seat = v_hub + w_frame x r_seat
% % The seat is located distance 'd' up the fork
% r_seat_rel_world = R_yaw * R_roll * R_pitch * [0; 0; d];
% vel_seat_world   = vel_hub_world + cross(w_frame_world, r_seat_rel_world);

% % 3. ENERGIES
% % Kinetic
% T_trans_wheel = 0.5 * m_w * (vel_hub_world.' * vel_hub_world);
% T_trans_seat  = 0.5 * m_s * (vel_seat_world.' * vel_seat_world);

% % Rotational Kinetic (Wheel)
% % Wheel rotation is Frame Rotation + Wheel Spin relative to frame
% w_wheel_world = w_frame_world + R_yaw * R_roll * [0; dotOmega; 0];
% % Project to local wheel frame for inertia tensor
% w_wheel_local = (R_yaw * R_roll).' * w_wheel_world; 
% local_wheel_inertias = diag([I_wyawroll, I_wpitch, I_wyawroll]);
% T_rot_wheel = 0.5 * w_wheel_local.' * local_wheel_inertias * w_wheel_local;

% T = T_trans_wheel + T_trans_seat + T_rot_wheel;

% % Potential (Height of center of mass)
% % Hub height is fixed at 'r' (assuming flat floor)
% % Seat height depends on lean (phi) and pitch (alpha)
% h_seat = r + r_seat_rel_world(3); 
% U = m_s * g * h_seat;

% L = simplify(T - U);

% %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% %                  evaluate partial derivatives                           %
% %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% %              DL  
% %  DL_Dq  ==  ---      Note that 'D' is partial derivative here 
% %              Dq
% %
% DL_Dq = jacobian(L,q)';

% %              DL  
% %  DL_Ddq  ==  ---      Note that 'D' is partial derivative here 
% %              Ddq
% %
% DL_Ddq = jacobian(L,dq);

% %                D  / DL  \         * Note that some of those 'd' should be
% % DDL_DtDdq  ==  -- | --  |         curvy 'D' to represent partial
% %                Dt \ Ddq /         derivatives
% %
% % Note the application of the chain rule:  (Quoting Andy Ruina: )
% %      d BLAH / dt  =  jacobian(BLAH, [q qdot])*[qdot qddot]'
% %
% DDL_DtDdq = jacobian(DL_Ddq,[q; dq]) * [dq; ddq];


% %Write out as single equation and simplify:
% EoM = simplify(DDL_DtDdq - DL_Dq);
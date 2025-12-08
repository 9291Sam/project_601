% syms x(t) y(t) theta(t) phi(t) Omega(t) alpha(t);

% q = [x(t); y(t); theta(t); phi(t); Omega(t); alpha(t)];
% dq = diff(q, t);
% ddq = diff(dq, t);


% % r          - wheel radius
% % d          - distance between fork and hub
% % m_w        - wheel mass
% % m_s        - seat mass
% % I_wpitch   - Wheel pitch moment (rolling)
% % I_wyawroll - Wheel not pitch moment
% % g          - gravitational constant
% syms r d m_w m_s I_wpitch I_wyawroll g 'real'; 

% local_wheel_inertias = [I_wyawroll 0 0; 0 I_wpitch 0; 0 0 I_wyawroll];

% % Order of rotations. 
% % {alpha / omega} -> phi -> theta -> world

% % these matrices bring things into global space from the local spaces
% Rz_theta = [
%     cos(theta) -sin(theta) 0; 
%     sin(theta) cos(theta)  0;
%     0          0           1
% ];

% rotate_by_theta = Rz_theta;

% Rx_phi = [
%     1 0        0;
%     0 cos(phi) -sin(phi);
%     0 sin(phi) cos(phi);
% ];

% rotate_by_phi = Rx_phi;

% Ry_alpha = [
%     cos(alpha)  0 sin(alpha);
%     0           1 0;
%     -sin(alpha) 0 cos(alpha);
% ];

% rotate_by_alpha = Ry_alpha;

% % global origin -> unicycle_base
% root = [x; y; 0];

% % vector between global origin -> wheel center
% wheel_hub_global_pos = root + rotate_by_theta * rotate_by_phi * [0; 0; r];
% % global origin -> seat center
% seat_global_pos = wheel_hub_global_pos + rotate_by_theta * rotate_by_phi * rotate_by_alpha * [0; 0; d];

% wheel_hub_global_velocity = diff(wheel_hub_global_pos, t);
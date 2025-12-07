clc; clear; close all;

f = figure();
clf;

grid on;
hold on;
axis equal;
axis([-8 8 -8 8 0 8]); 
axis manual; 
axis vis3d;

view(3);            
xlabel('X'); ylabel('Y'); zlabel('Z');
plot3(0, 0, 0, 'kx', 'MarkerSize', 15, 'LineWidth', 2);

wheel_center_seat_distance = 2.5;
wheel_radius = 1.0;
wheel_width = 0.25;
simulation_delta_time = 1/60;

unicycle_root_transform = hgtransform; 

% Seat Line
seat_transform = hgtransform('Parent', unicycle_root_transform);
plot3([0, 0], [0, 0], [0, wheel_center_seat_distance], 'LineWidth', 4, 'Color', 'r', 'Parent', seat_transform);

% Wheel
wheel_transform = hgtransform('Parent', unicycle_root_transform);
wheel_xrotated_pi2 = hgtransform('Parent', wheel_transform);
set(wheel_xrotated_pi2, 'Matrix', makehgtform('xrotate', pi/2));
[xc, yc, zc] = cylinder(wheel_radius, 30);
zc = zc * wheel_width - (wheel_width/2);
surf(xc, yc, zc, 'Parent', wheel_xrotated_pi2, 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none'); 
% Wheel Spoke
plot3([0 0], [0 wheel_radius], [0 0], 'LineWidth', 3, 'Color', 'b', 'Parent', wheel_xrotated_pi2);


t = 0;
pathRadius = 6.0;

% Inputs: 
% wheel torque        | τ_1
% lean side torque    | τ_2  
% lean forward torque | τ_3

% Outputs:
% wheel position        | x y
% wheel pitch (rolling) | Ω
% wheel yaw             | θ
% wheel roll            | φ
% shaft lean            | α


while ishandle(f)
    x = pathRadius * cos(t);
    y = pathRadius * sin(t);
    theta = t + (pi/2); % tangent to a circle
    phi = -0.3;
    Omega = -(pathRadius * t) / wheel_radius;
    alpha = -0.3;
    
    
    T_pos  = makehgtform('translate', [x, y, wheel_radius]);
    T_yaw  = makehgtform('zrotate', theta);
    T_lean = makehgtform('xrotate', phi);
    
    % Handles x, y, theta, and phi
    set(unicycle_root_transform, 'Matrix', T_pos * T_yaw * T_lean);
    
    set(wheel_transform, 'Matrix', makehgtform('yrotate', Omega));

    set(seat_transform, 'Matrix',  makehgtform('yrotate', alpha))
    
    drawnow; t = t + simulation_delta_time; pause(simulation_delta_time);
end
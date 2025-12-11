function draw(sys, x)

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



wheel_center_seat_distance = sys.d;
wheel_radius = sys.r;
wheel_width = 0.25;

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


last_valid_k = 1;

for k = 1:size(x,1)

    % if (abs(x(k, 4)) < pi/2 && abs(x(k, 6)) < pi/2)
        last_valid_k = k;
    % end


    % rendering
    x_pos = x(last_valid_k, 1);
    y = x(last_valid_k, 2);
    theta = x(last_valid_k, 3); % tangent to a circle
    phi = x(last_valid_k, 5);
    Omega = x(last_valid_k, 10);
    alpha = x(last_valid_k, 7);
    
    T_contact = makehgtform('translate', [x_pos, y, 0]);
    T_yaw  = makehgtform('zrotate', theta);
    T_lean = makehgtform('xrotate', phi);
    T_hub_offset = makehgtform('translate', [0, 0, wheel_radius]);
    
    % Handles x, y, theta, and phi
    set(unicycle_root_transform, 'Matrix', T_contact * T_yaw * T_lean * T_hub_offset);
    
    set(wheel_transform, 'Matrix', makehgtform('yrotate', Omega));

    set(seat_transform, 'Matrix',  makehgtform('yrotate', alpha))
    
    disp("rendering state");
    x(k, :)
    drawnow; pause(1/120)
end



end
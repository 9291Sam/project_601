function draw(sys, x)
    f = figure("Position", [100, 100, 1920, 1620]);
    clf;

    grid on;
    hold on;
    axis equal;
    axis([-4 4 -4 4 0 4]); 
    axis manual; 
    axis vis3d;

    view(3);            
    xlabel('X'); ylabel('Y'); zlabel('Z');
    plot3(0, 0, 0, 'kx', 'MarkerSize', 15, 'LineWidth', 2); % root marker

    ball_radius = sys.r;
    body_length = sys.d * 2;

    ballbot_root_transform = hgtransform; 
    body_transform = hgtransform('Parent', ballbot_root_transform);


    % Ball geometry
    [sx, sy, sz] = sphere(20);
    sx = sx * ball_radius;
    sy = sy * ball_radius;
    sz = sz * ball_radius;
    surf(sx, sy, sz, 'Parent', ballbot_root_transform, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', [0.5 0.5 0.5]);

    % Body geometry　% TODO: should this be a cylinder
    plot3([0, 0], [0, 0], [0, body_length], 'LineWidth', 6, 'Color', 'r', 'Parent', body_transform);

    for k = 1:size(x,1)
        x_pos = x(k, 1);
        y_pos = x(k, 2);
        theta = x(k, 3); 
        phi   = x(k, 4);         
        
        T_translate = makehgtform('translate', [x_pos, y_pos, ball_radius]);
        set(ballbot_root_transform, 'Matrix', T_translate);
        
        T_pitch = makehgtform('yrotate', theta);
        T_roll  = makehgtform('xrotate', phi);
        
        set(body_transform, 'Matrix', T_pitch * T_roll);
        
        drawnow; 
        
        pause(1/60); 
    end
end
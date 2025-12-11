function dx = dynamics_no_input(sys, x)
    % x is 10x1 state vector: 
    % [x_pos; y_pos; theta; v; phi; dphi; alpha; dalpha; dtheta; Omega]

    % 1. Unpack State
    theta  = x(3);
    v      = x(4);
    phi    = x(5);
    dphi   = x(6);
    alpha  = x(7);
    dalpha = x(8);
    dtheta = x(9);
    
    % Group into vectors for the helper functions
    % Note: dq order must match eom_deriver definition: [dtheta; dphi; dalpha]
    q  = [theta; phi; alpha];
    dq = [dtheta; dphi; dalpha];

    % 2. Solve Drivers (v and Omega) with u=0
    u_wheel = 0;
    dv = getDriverAcceleration(u_wheel, sys.r, sys.m_w, sys.m_s);
    dOmega = v / sys.r;

    % 3. Get Mass Matrix and Force Vector
    % We pass 'dv' to FVector because gyroscopic terms depend on linear acceleration
    M = getMassMatrix(q, dq, v, sys.r, sys.d, sys.m_w, sys.m_s, sys.I_wpitch, sys.I_wyawroll, sys.g);
    f = getFVector(q, dq, v, dv, sys.r, sys.d, sys.m_w, sys.m_s, sys.I_wpitch, sys.I_wyawroll, sys.g);

    % 4. Solve for Angular Accelerations
    % Equation: M * ddq + f = 0   (since u=0)
    % ddq = -M \ f
    ddq = -M \ f;
    
    ddtheta = ddq(1);
    ddphi   = ddq(2);
    ddalpha = ddq(3);

    % 5. Pack Derivative Vector (dx)
    dx = zeros(10, 1);
    
    dx(1) = v * cos(theta); % dx_pos
    dx(2) = v * sin(theta); % dy_pos
    dx(3) = dtheta;         % dtheta
    dx(4) = dv;             % dv
    dx(5) = dphi;           % dphi
    dx(6) = ddphi;          % ddphi (acceleration)
    dx(7) = dalpha;         % dalpha
    dx(8) = ddalpha;        % ddalpha (acceleration)
    dx(9) = ddtheta;        % ddtheta (acceleration)
    dx(10)= dOmega;         % dOmega
end
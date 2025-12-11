function dx = dynamics_with_input(sys, t, x, control_fun)
    % 1. UNPACK STATE (10x1)
    % [x; y; theta; v; phi; dphi; alpha; dalpha; dtheta; Omega]
    theta  = x(3);
    v      = x(4);
    phi    = x(5);
    dphi   = x(6);
    alpha  = x(7);
    dalpha = x(8);
    dtheta = x(9);
    
    % Group for generated functions
    q  = [theta; phi; alpha];
    dq = [dtheta; dphi; dalpha];

    % 2. GET CONTROL INPUT
    if nargin > 3 && ~isempty(control_fun)
        u = control_fun(t, x);
    else
        u = [0; 0];
    end
    u_wheel = u(1);
    
    % 3. SOLVE DRIVERS (Forward Acceleration)
    % This uses F=ma for the linear motion
    dv = getDriverAcceleration(u_wheel, sys.r, sys.m_w, sys.m_s);
    dOmega = v / sys.r;

    % 4. GET MATRICES (Lagrangian Solver)
    % M depends on state, f depends on state+accel
    M = getMassMatrix(q, dq, v, sys.r, sys.d, sys.m_w, sys.m_s, sys.I_wpitch, sys.I_wyawroll, sys.g);
    f = getFVector(q, dq, v, dv, sys.r, sys.d, sys.m_w, sys.m_s, sys.I_wpitch, sys.I_wyawroll, sys.g);
    
    % FIXED: getInputMatrix() takes NO arguments because B is constant
    B = getInputMatrix(); 

    % 5. SOLVE DYNAMICS
    % M * ddq + f = B * u   =>   ddq = M \ (B*u - f)
    ddq = M \ (B * u - f);
    
    ddtheta = ddq(1);
    ddphi   = ddq(2);
    ddalpha = ddq(3);

    % 6. PACK DERIVATIVES
    dx = zeros(10, 1);
    dx(1) = v * cos(theta); % dx
    dx(2) = v * sin(theta); % dy
    dx(3) = dtheta;         % dtheta
    dx(4) = dv;             % dv
    dx(5) = dphi;           % dphi
    dx(6) = ddphi;          % ddphi
    dx(7) = dalpha;         % dalpha
    dx(8) = ddalpha;        % ddalpha
    dx(9) = ddtheta;        % ddtheta
    dx(10)= dOmega;         % dOmega
end
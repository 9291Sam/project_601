classdef ballbot
    % A Ballbot is an 8-dimension nonlinear system with state
    % [x; y; theta; phi; dx; dy; dtheta; dphi]
    % where
    % x - cartesian position of the contact point of the base of the ball
    % y - cartesian position of the contact point of the base of the ball
    % theta - rotation around the global y axis. +theta leans towards +X
    % phi   - rotation around the global z axis. +phi leans towards -Y

    % with constants
    % r         - ball radius
    % d         - distance between ball com & body com
    % m_ball    - mass of the ball
    % m_body    - mass of the body
    % I_ball    - moment of inertia of the ball (spherically symmetrical)
    % I_body_xy - moment of inertia of the body around the x&y axis
    % I_body_z  - moment of inertia of the body around the z axis
    % g         - gravitational constant

    properties
        num_states
        num_inputs
        r       
        d       
        m_ball  
        m_body  
        I_ball  
        I_body_xy
        I_body_z
        g       
    end

    methods
        function obj = ballbot()
            obj.num_states = 8;
            obj.num_inputs = 2;

            obj.r = 0.1;
            obj.d = 0.5;
            obj.m_ball = 2.0;
            obj.m_body = 10.0;
            obj.I_ball = (2/5) * obj.m_ball * obj.r^2
            radius_body = obj.r * 1.25;
            length_body = obj.d * 2;
            obj.I_body_xy = (1/4) * obj.m_body * radius_body^2 + (1/12) * obj.m_body * length_body^2
            obj.I_body_z = (1/2) * obj.m_body * radius_body^2
            obj.g = 10;
        end
    end
end
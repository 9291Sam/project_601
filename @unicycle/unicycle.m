classdef unicycle
    % The unicycle is a 12-dimensional nonlinear system with state
    % [x; y; theta; phi; Omega; alpha; dotx; doty; dottheta; dotphi; omega; dotalpha]
    % where:
    %   x - Cartesian coordinate of the contact point of the unicycle's wheel to the ground
    %   y - Cartesian coordinate of the contact point of the unicycle's wheel to the ground
    %   theta - Angle between the unicycle's forward (+x) vector and the global (+x) vector
    %   phi - Angle between the unicycle's vertical (+z) vector and the global
    %   Omega - Angle spun by the wheel
    %   alpha - Angle of the fork

    % Order of Rotations
    % {alpha, Omega} -> phi -> theta

    properties
        num_states
        num_inputs
        r
        d
        m_w
        m_s
        I_wpitch
        I_wyawroll
        g
    end

    methods
        function obj = unicycle()
            obj.num_states = 12;
            obj.num_inputs = 0;

            obj.r = 0.5;
            obj.d = 1.25;
            obj.m_w = 5.0;
            obj.m_s = 60;
            obj.I_wpitch = 0.5 * obj.m_w * obj.r^2; % TODO: numbers pulled from the ether
            obj.I_wyawroll = 0.25 * obj.m_w * obj.r^2;
            obj.g = 10;

            % eom_deriver();
        end
    end

    methods (Static)
        eom_deriver()
    end
end
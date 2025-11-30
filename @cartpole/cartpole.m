classdef cartpole
    % The cartpole is a 4-dimensional nonlinear system with state
    % [ x; dx; theta; dtheta ]
    % where x is the position of the cart and theta is the angle of the pole
    
    properties
        num_states      % number of states
        num_inputs      % number of inputs
        mass_pendulum   % mass of the pendulum
        mass_cart       % mass of the cart
        length_pole     % length of the pole
        gravity         % acceleration due to gravity
        damping_cart    % damping of the cart
    end
    
    methods
        
        % constructor
        function obj = cartpole()
            
            obj.num_states = 4;
            obj.num_inputs = 1;
            
            % default parameters
            obj.mass_pendulum   = 1;
            obj.mass_cart       = 5;
            obj.length_pole     = 2;
            obj.gravity         = -10;
            obj.damping_cart    = 7;
        end
    end
end
function dx = dynamics(sys, state, input)

if nargin < 2
    state = zeros(sys.num_states,1);
end
if nargin < 3
    input = zeros(sys.num_inputs,1);
end

    new_states = ballbot_dynamics(...
        state(1), state(2), state(3), state(4), state(5), state(6), state(7), state(8),...
        input(1), input(2),...
        sys.r, sys.d, sys.m_ball, sys.m_body, sys.I_ball, sys.I_body_xy, sys.I_body_z, sys.g...
    );

    dx = [state(5:8); new_states];
end
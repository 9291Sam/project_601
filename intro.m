%% Cart-pole

% The cartpole is a 4-dimensional nonlinear system with state
% [ x; dx; theta; dtheta ]
% where x is the position of the cart and theta is the angle of the pole

% Based on the demo by Steve Brunton
%  - YouTube video: https://www.youtube.com/watch?v=qjhAAQexzLg
%  - GitHub code: https://github.com/bertozzijr/Control_Bootcamp_S_Brunton

% Triple pendulum on a cart (https://www.youtube.com/watch?v=meMWfva-Jio)


%% Simulate the system starting at the unstable equilibrium
clc; clear; close all;

% % % cart-pole system
% sys = cartpole();

% % initial condition (vertical)
% x0 = [0; 0; pi; 0];

% % time vector
% t = 0:0.1:60;

% % simulate the system
% x = sys.simulate(t,x0);

% % visualize the dynamics
% sys.draw(x);

% sys = unicycle();

% % Initial Conditions
% x0 = zeros(10, 1);
% x0(1) = 0; % global x
% x0(2) = 0; % global y
% x0(3) = 0; % theta
% x0(4) = 0; % v
% x0(5) = 0; % phi
% x0(6) = 0; % dphi
% x0(7) = 0.1; % alpha
% x0(8) = 0; % dalpha
% x0(9) = 0; % dtheta
% x0(10) = 0; % omega

% % Simulation Time
% t = 0:0.01:5;

% x = sys.simulate_no_input(t, x0);

% sys.draw(x);
% 1. Setup
sys = unicycle();
t_span = 0:0.1:25;

% 2. Linearize around SLOWER Equilibrium
x_eq = zeros(10, 1);
x_eq(4) = 1.5; % Slow down to 1.5 m/s for tighter turns
u_eq = [0; 0];

[A, B] = sys.linearize(x_eq, u_eq);
[A, B] = sys.linearize(x_eq, u_eq);

% 3. Slice Matrices for LQR (FIXED: REMOVED STATE 9)
% We keep: v(4), phi(5), dphi(6), alpha(7), dalpha(8)
idx_ctrl = [4, 5, 6, 7, 8]; 

A_lqr = A(idx_ctrl, idx_ctrl);
B_lqr = B(idx_ctrl, :);

% 4. Design LQR Controller
idx_ctrl = [4, 5, 6, 7, 8]; 
A_lqr = A(idx_ctrl, idx_ctrl);
B_lqr = B(idx_ctrl, :);

Q_diagonals = [
    10    % v (Velocity)
    500   % phi (Roll Angle - CRITICAL: much higher gain to force the lean)
    1     % dphi 
    100   % alpha (Pitch Stability)
    10    % dalpha
];
Q = diag(Q_diagonals);
R = diag([1, 10]); 

K = lqr(A_lqr, B_lqr, Q, R);

% 5. Simulation
% We wrap the controller to inject the Figure-8 logic
controller = @(t, x) figure_8_logic(t, x, K, x_eq);

x0 = x_eq; 
% Add a tiny perturbation so it doesn't get stuck in the 'perfect' equilibrium
x0(5) = 0.001; 

% ... [Setup and LQR calculation from previous step] ...

% 5. Simulation
% Use the corrected dynamics_with_input function
[t_out, x_out] = ode45(@(t,x) sys.dynamics_with_input(t, x, controller), t_span, x0);

% 6. Visualize
sys.draw(x_out);


function u = figure_8_logic(t, x, K, x_eq)
    % Extract control states
    idx_ctrl = [4, 5, 6, 7, 8];
    x_sub = x(idx_ctrl);
    
    target_v = x_eq(4); % Now 1.5 m/s
    
    % --- TIGHTER FIGURE 8 ---
    % Faster switching (10s total cycle) + Steeper Lean (0.35 rad)
    cycle_time = 5; 
    lean_amount = 0.35; % ~20 degrees
    
    if mod(t, cycle_time) < cycle_time/2
        phi_ref = lean_amount;  % Turn Right
    else
        phi_ref = -lean_amount; % Turn Left
    end
    
    % Reference State
    ref_state = [target_v; phi_ref; 0; 0; 0];
    
    error = x_sub - ref_state;
    u = -K * error;
    
    % Clamp Inputs
    u = max(min(u, [100; 50]), -[100; 50]);
end
%% Control via pole placement
% clc; clear; close all;

% % cart-pole system
% sys = cartpole();

% % time vector
% t = 0:0.1:20;

% % equilibrium for 'up' position
% xtilde = [3 0.0 pi 0]';
% utilde = 0;

% % initial condition (slightly left of vertical)
% x0 = [-3 0 pi+0.1 0]';

% % linearization
% [A,B] = sys.linearize(xtilde,utilde);

% % desired eigenvalues
% % p = [-.01; -.02; -.03; -.04]; % not enough
% % p = [-.3; -.4; -.5; -.6];     % just barely
% p = [-1; -1.1; -1.2; -1.3];   % good
% % p = [-2; -2.1; -2.2; -2.3];   % aggressive
% % p = [-3; -3.1; -3.2; -3.3];   % aggressive
% % p = [-3.5; -3.6; -3.7; -3.8]; % breaks

% % place the closed-loop eigenvalues
% K = place(A,B,p);

% % simulate the system
% x = sys.static_state_feedback(t,x0,K,xtilde,utilde);

% % visualize the dynamics
% sys.draw(x);


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



sys = unicycle();

initial_velocity = 1.5; 
initial_omega = initial_velocity / sys.r; 

x0 = zeros(12, 1);
% x0(8) = 0.5;
x0(3) = 5*pi/4; % theta
x0(4) = 0.0001; % phi
% x0(9) = -0.5; % theta dot
x0(11) = initial_omega; % Set initial wheel speed
x0(7) = initial_velocity * cos(x0(3)); % Set initial x velocity
x0(8) = initial_velocity * sin(x0(3)); % Set initial y velocity

t = 0:0.1:60;

x = sys.simulate_no_input(t, x0);

sys.draw(x);

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


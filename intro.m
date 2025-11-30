%% Cart-pole

% The cartpole is a 4-dimensional nonlinear system with state
% [ x; dx; theta; dtheta ]
% where x is the position of the cart and theta is the angle of the pole

% Based on the demo by Steve Brunton
%  - YouTube video: https://www.youtube.com/watch?v=qjhAAQexzLg
%  - GitHub code: https://github.com/bertozzijr/Control_Bootcamp_S_Brunton

% Triple pendulum on a cart (https://www.youtube.com/watch?v=meMWfva-Jio)


%% Simulate the system starting at the unstable equilibrium
% clc; clear; close all;

% % cart-pole system
% sys = cartpole();

% % initial condition (vertical)
% x0 = [0; 0; pi; 0];

% % time vector
% t = 0:0.1:60;

% % simulate the system
% x = sys.simulate(t,x0);

% % visualize the dynamics
% sys.draw(x);


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


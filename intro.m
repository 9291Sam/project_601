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
% sys = ballbot();

% % initial condition (vertical)
% x0 = [0; 0; 0.001; 0.001; 0; 0; 0; 0];
% % u = [0; 0]

% % time vector
% t = 0:0.1:60;

% % simulate the system
% x = sys.simulate(t,x0);

% % visualize the dynamics
% sys.draw(x);

%% Control via lqr

% sys = ballbot();

% % time vector
% t = 0:0.1:20;

% % equilibrium for 'up' position
% xtilde = [3; 2; 0; 0; 0; 0; 0; 0];
% utilde = [0; 0];

% % initial condition (slightly left of vertical)
% x0 = [-3; 0; 0.5; -.6; 0; 0; 0; 0];

% % linearization
% [A,B] = sys.linearize(xtilde,utilde);

% [K, ~, ~] = lqr(A, B, diag([1, 1, 100, 100, 1, 1, 1, 1]), [1 0; 0 1]);

% % simulate the system
% x = sys.static_state_feedback(t,x0,K,xtilde,utilde);

% % visualize the dynamics
% sys.draw(x);


%% control in a loop
sys = ballbot();

% time vector
t = 0:0.1:20;

% equilibrium for 'up' position
xtilde = [3; 2; 0; 0; -10; 0; 0; 0];
utilde = [0; 0];

% initial condition (slightly left of vertical)
x0 = [-3; 0; 0.5; -.6; 0; 0; 0; 0];

radius = 2.0;
period = 5.0;
w = 2 * pi / period;
t_span = 0:0.05:20;
r = @(t) [
    radius * cos(w*t);       
    radius * sin(w*t);     
    0; 
    0;
    -radius * w * sin(w*t);
    radius * w * cos(w*t);
    0;
    0 
];

% linearization
[A,B] = sys.linearize(xtilde,utilde);

[K, ~, ~] = lqr(A, B, diag([10, 10, 1, 1, 10, 10, 1, 1]), [1 0; 0 1]);

% simulate the system
x = sys.trajectory_tracked_static_state_feedback(t_span,x0,K,r);

% visualize the dynamics
sys.draw(x);
%% control from observer

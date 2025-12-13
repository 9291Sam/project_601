%% Simulate the system starting at the unstable equilibrium
% clc; clear; close all;
% sys = ballbot();

% % initial condition (vertical)
% x0 = [0; 0; 0.00001; 0.0001; 0; 0; 0; 0];
% % u = [0; 0]

% % time vector
% t = 0:0.01:15;

% % simulate the system
% x = sys.simulate(t,x0);

% % visualize the dynamics
% % sys.draw(x);


% s = 1

% figure("Position", [100, 100, 1280, 720]);
% set(gcf, 'Color', 'w');
% % refs = r(t_span);
% plot(t, x(:, 7)', "r"); hold on
% % plot(t_span, refs(s, :), 'b'); hold on
% % plot(t, x(:, 3), 'g'); hold on

% legend("dottheta")
% exportgraphics(gcf, 'uncontrolled_dottheta_no_deviation.png'); 



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
% sys = ballbot();

% % time vector
% t = 0:0.1:20;

% linearization_x = [3; 2; 0; 0; 0; 0; 0; 0];
% utilde = [0; 0];

% % initial condition 
% x0 = [-3; 0; 0.5; -.6; 0; 0; 0; 0];

% radius = 3.0;
% period = 5.0;
% w = 2 * pi / period;
% t_span = 0:0.05:20;
% r = @(t) [
%     radius * cos(w*t);       
%     radius/2 * sin(2*w*t);     
%     0; 
%     0;
%     0;
%     0;
%     % -radius * w * sin(w*t);
%     % radius * w * cos(2*w*t);
%     0;
%     0 
% ];

% % linearization
% [A,B] = sys.linearize(linearization_x,utilde);

% [K, ~, ~] = lqr(A, B, diag([10, 10, 1, 1, 1, 1, 1, 1]), [1 0; 0 1]);

% % simulate the system
% x = sys.trajectory_tracked_static_state_feedback(t_span,x0,K,r);

% % visualize the dynamics
% sys.draw(x);
%% control from observer
sys = ballbot();

% time vector
linearization_x = [3; 2; 0; 0; 0; 0; 0; 0];
utilde = [0; 0];

% initial condition 
x0_plant = [-3; -1.33; 0.5; -.6; 0; 0; 0; 0];
x0_observer = zeros(size(x0_plant));

% x0 = [x0_plant; x0_plant - [0; 0; 0 - 0.1; 0; 0; 0; 0; 0]];
x0 = [x0_plant; x0_observer];

radius = 3.0;
period = 5.0;
w = 2 * pi / period;
t_span = 0:0.025:22;
r = @(t) [
    radius * cos(w*t);       
    radius/2 * sin(2*w*t);     
    0*t; 
    0*t;
    0*t;
    0*t;
    0*t;
    0*t 
];

% linearization
[A,B] = sys.linearize(linearization_x,utilde);

% controller
[K, ~, ~] = lqr(A, B, diag([10, 10, 1, 1, 1, 1, 1, 1]), eye(2));

C = [
    1, 0, 0, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0, 0, 0;
    % 0, 0, 1, 0, 0, 0, 0, 0;
    % 0, 0, 0, 1, 0, 0, 0, 0;
    % 0, 0, 0, 0, 1, 0, 0, 0;
    % 0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 0, 1;
];
[rows, columns] = size(C);
D = zeros(rows, 2);

[L_T, ~, ~] = lqr(A', C', 10 * diag([1, 1, 25, 25, 1, 1, 5, 5]), eye(rows));
L = L_T';


% simulate the system
x = sys.observed_trajectory_tracked(t_span,x0,K,r, L, C, D);

s = 8;
 
figure("Position", [100, 100, 1280, 720]);
set(gcf, 'Color', 'w');
refs = r(t_span);
plot(t_span, x(:, s)', "r"); hold on
plot(t_span, refs(s, :), 'b'); hold on
plot(t_span, x(:, s+8), 'g'); hold on

legend("Actual dottheta", "Desired dottheta", "Observed dottheta")
exportgraphics(gcf, 'final_controlled_dottheta.png'); 


% visualize the dynamics
% sys.draw(x);

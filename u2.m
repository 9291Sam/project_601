clc; clear; close all;

%% --- Visualization Setup ---
f = figure('Name', 'Unicycle Dynamics Simulation', 'Color', 'w');
clf;

grid on;
hold on;
axis equal;
axis([-4 4 -4 4 0 4]); 
axis manual; 
axis vis3d;

view(3);            
xlabel('X'); ylabel('Y'); zlabel('Z');
% Ground
patch([-10 10 10 -10], [-10 -10 10 10], [0 0 0 0], [0.9 0.9 0.9], 'EdgeColor', 'none');

% Define Physical Parameters
params.r = 0.5;      % Wheel Radius (meters)
params.d = 1.25;      % Distance from wheel center to seat mass (meters)
params.mw = 5.0;     % Mass of wheel (kg)
params.mr = 50.0;    % Mass of rider/frame (kg)
params.g = 9.81;     % Gravity
params.Iwy = 0.5 * params.mw * params.r^2;   % Wheel Inertia Y (Spin axis)
params.Iwz = 0.25 * params.mw * params.r^2;  % Wheel Inertia Z (Yaw axis)
params.wheel_width = 0.25;

% Graphics Transforms
unicycle_root_transform = hgtransform; 

% Seat/Frame Graphics
seat_transform = hgtransform('Parent', unicycle_root_transform);
% Visualizing the rod
plot3([0, 0], [0, 0], [0, params.d], 'LineWidth', 4, 'Color', 'r', 'Parent', seat_transform);


% Wheel Graphics
wheel_transform = hgtransform('Parent', unicycle_root_transform);
wheel_xrotated_pi2 = hgtransform('Parent', wheel_transform);
set(wheel_xrotated_pi2, 'Matrix', makehgtform('xrotate', pi/2));
[xc, yc, zc] = cylinder(params.r, 30);
zc = zc * params.wheel_width - (params.wheel_width/2);
surf(xc, yc, zc, 'Parent', wheel_xrotated_pi2, ...
    'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none'); 
% Wheel Spoke for rotation visualization
plot3([0 0], [0 params.r], [0 0], 'LineWidth', 3, 'Color', 'b', 'Parent', wheel_xrotated_pi2);

%% --- Simulation Setup ---

% Time settings
fps = 60;
dt = 1/fps;
t_current = 0;

% Initial State Vector (12x1)
% Indices:
% 1: x      7: dx
% 2: y      8: dy
% 3: theta  9: dtheta (Yaw Rate)
% 4: phi   10: dphi   (Roll Rate)
% 5: beta  11: omega  (Wheel Spin Rate)
% 6: alpha 12: dalpha (Pitch Rate)

initial_velocity = 0.5; % Start with some speed so it doesn't flop instantly
initial_omega = initial_velocity / params.r; 

state = zeros(12, 1);
state(8) = 0.5;
% state(3) = 5*pi/4; % theta
% state(4) = 0.0001; % phi
% state(9) = -0.5; % theta dot
% state(11) = initial_omega; % Set initial wheel speed
% state(7) = initial_velocity * cos(state(3)); % Set initial x velocity
% state(8) = initial_velocity * sin(state(3)); % Set initial y velocity

% Pre-allocate for loop
t_span = [0 dt];

%% --- Main Loop ---
while ishandle(f)
    
    % Integrate physics for one time step
    [ts, states_out] = ode45(@(t, s) unicycle_dynamics(t, s, params), [t_current, t_current + dt], state);
    
    % Update state for next iteration
    state = states_out(end, :)'
    t_current = t_current + dt;
    
    % --- Unpack State for Visualization ---
    posX = state(1);
    posY = state(2);
    yaw = state(3);    % Theta
    roll = state(4);   % Phi (Lean)
    wheelAngle = state(5); 
    pitch = state(6);  % Alpha (Frame tilt)
    
    % --- Update Transforms ---
    
    % 1. Root Position and Heading
    % Note: Z position is fixed to wheel radius assuming flat ground
    T_pos  = makehgtform('translate', [posX, posY, 0]);
    T_yaw  = makehgtform('zrotate', yaw);
    T_roll = makehgtform('xrotate', roll);
    T_offset  = makehgtform('translate', [0, 0, params.r]);
    
    % Apply to root: Position -> Yaw -> Roll
    set(unicycle_root_transform, 'Matrix', T_pos * T_yaw * T_roll * T_offset);
    
    % 2. Wheel Spin (Relative to Root)
    set(wheel_transform, 'Matrix', makehgtform('yrotate', wheelAngle));

    % 3. Seat Pitch (Relative to Root)
    set(seat_transform, 'Matrix',  makehgtform('yrotate', pitch));
    
    % Draw
    drawnow; 
    
    % Basic check to stop if it falls over completely
    if abs(roll) > pi/2 || abs(pitch) > pi/2
        % text(0, 0, 4, 'CRASHED', 'FontSize', 20, 'HorizontalAlignment', 'center');
        break;
    end
end

%% --- Dynamics Function ---
function dState = unicycle_dynamics(~, state, p)
    % Unpack State
    % x = state(1);
    % y = state(2);
    theta = state(3);
    phi   = state(4);
    % beta  = state(5);
    alpha = state(6);
    
    % Velocities
    % dx     = state(7);
    % dy     = state(8);
    dtheta = state(9);
    dphi   = state(10);
    omega  = state(11);
    dalpha = state(12);
    
    % Inputs (Torques)
    % To make this controllable, you would calculate these using a controller
    % For now, they are zero (passive physics)
    tau_1 = 0; % Wheel Torque
    tau_2 = 0; % Roll Torque
    tau_3 = 0; % Pitch Torque (between frame and wheel)
    
    % --- Equations of Motion (from LaTeX) ---
    
    % Common terms for readability
    sin_phi = sin(phi); cos_phi = cos(phi);
    sin_theta = sin(theta); cos_theta = cos(theta);
    sin_alpha = sin(alpha); cos_alpha = cos(alpha);
    
    % Row 10: Phi double dot (Roll Acceleration)
    % Numerator
    num_phi = (p.mw*p.r + p.mr*(p.r + p.d)) * (p.g*sin_phi + p.r*omega*dtheta*cos_phi) ...
              - p.Iwz*omega*dtheta*cos_phi + tau_2;
    % Denominator
    den_phi = p.mw * p.r^2 + p.mr * (p.r + p.d)^2 + p.Iwy;
    
    ddphi = num_phi / den_phi;
    
    % Coupled Dynamics for Omega (Wheel Accel) and Alpha (Pitch Accel)
    % Denominator is shared for Row 11 and 12
    den_coupled = ( (p.mw + p.mr)*p.r^2 + p.Iwy ) * (p.mr * p.d^2) ...
                  - (p.mr * p.r * p.d * cos_alpha)^2;
    
    % Row 11: Omega dot (Wheel Angular Acceleration)
    num_omega = (p.mr * p.d^2) * tau_1 ...
                - (p.mr * p.r * p.d * cos_alpha) * (p.mr * p.g * p.d * sin_alpha - tau_1 + tau_3);
            
    domega = num_omega / den_coupled;
    
    % Row 12: Alpha double dot (Pitch Acceleration)
    num_alpha = ((p.mw + p.mr)*p.r^2 + p.Iwy) * (p.mr * p.g * p.d * sin_alpha - tau_1 + tau_3) ...
                - (p.mr * p.r * p.d * cos_alpha) * tau_1;
            
    ddalpha = num_alpha / den_coupled;
    
    % --- Kinematic Constraints Derivatives ---
    
    % Row 9: Theta double dot
    ddtheta = domega * sin_phi + omega * dphi * cos_phi;
    
    % Row 7: X double dot
    ddx = p.r * (domega * cos_theta - omega * dtheta * sin_theta);
    
    % Row 8: Y double dot
    ddy = p.r * (domega * sin_theta + omega * dtheta * cos_theta);
    
    % --- Assemble Final State Update Vector ---
    dState = [
        state(7);   % dx
        state(8);   % dy
        state(9);   % dtheta
        state(10);  % dphi
        omega;      % d(beta) = omega
        state(12);  % dalpha
        ddx;        % d(dx)
        ddy;        % d(dy)
        ddtheta;    % d(dtheta)
        ddphi;      % d(dphi)
        domega;     % d(omega)
        ddalpha     % d(dalpha)
    ];
end
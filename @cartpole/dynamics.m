function dx = dynamics(sys,x,u)

if nargin < 2
    x = zeros(sys.num_states,1);
end
if nargin < 3
    u = zeros(sys.num_inputs,1);
end

% parameters
m = sys.mass_pendulum;
M = sys.mass_cart;
L = sys.length_pole;
g = sys.gravity;
d = sys.damping_cart;

Sx = sin(x(3));
Cx = cos(x(3));
D  = m*L*L*(M+m*(1-Cx^2));

% system dynamics: dx = f(x,u)
dx(1,1) = x(2);
dx(2,1) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/D)*u;
dx(3,1) = x(4);
dx(4,1) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u;

end
function [A,B] = linearize(sys,x,u)

% parameters
m = sys.mass_pendulum;
M = sys.mass_cart;
L = sys.length_pole;
g = sys.gravity;
d = sys.damping_cart;

% linearized dynamics for 'down' position
if x(3) == 0
    
    A = [0 1 0 0;
        0 -d/M -m*g/M 0;
        0 0 0 1;
        0 d/(M*L) (m+M)*g/(M*L) 0];

    B = [0; 1/M; 0; -1/(M*L)];

% linearized dynamics for 'up' position
elseif x(3) == pi
    
    A = [0 1 0 0;
        0 -d/M -m*g/M 0;
        0 0 0 1;
        0 -d/(M*L) -(m+M)*g/(M*L) 0];

    B = [0; 1/M; 0; 1/(M*L)];
    
else
    error('invalid equilibrium');
end

end
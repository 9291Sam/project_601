function [x] = dynamic_output_feedback(sys,t,x0,L,K,xtilde,utilde,C,D)

% number of states
n = length(x0)/2;

% plant
plant = @(x,u) sys.dynamics(x,u);

% observer
observer = @(xhat,y,u) sys.dynamics(xhat,u) - L*(y-C*xhat-D*u);

% controller
controller = @(xhat) utilde - K*(xhat-xtilde);

% output
output = @(x,u) C*x;

% full dynamics
f = @(t,x) full_dynamics(plant,observer,controller,output,x(1:n),x(n+1:2*n));

% solve the differential equation
[~,x] = ode45(f,t,x0);

end


function f = full_dynamics(plant,observer,controller,output,x,xhat)

u = controller(xhat);
y = output(x,u);

f = [ plant(x,u); observer(xhat,y,u) ];

end
function [x] = static_state_feedback(sys,t,x0,K,xtilde,utilde)

u = @(x) utilde - K*(x - xtilde);

[~,x] = ode45(@(t,x) sys.dynamics(x,u(x)),t,x0);

end
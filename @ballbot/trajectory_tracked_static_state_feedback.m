function [x] = trajectory_tracked_static_state_feedback(sys,t_span,x0,K,r)

    u = @(t, x) -K*(x - r(t));

    [~,x] = ode45(@(t,x) sys.dynamics(x,u(t, x)),t_span,x0);

end
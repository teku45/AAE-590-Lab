function xdot=deriv(t,x,u)
% equations of motion
% input
% t = current time
% x = current state vector
% u = current control vector

% output
% xdot = derivative of x and y
global x02 
% constant parameter
g = 9.82;

V = sqrt(2*g*(x02-x(2)));
% evaluate equations of motion at current conditions
xdot(1) = V*cos(u);
xdot(2) = V*sin(u);

%
%   Simple ODE45 example
%       Author: Keitaro Naruse
%       Date: 2021-05-19
%

tspan = [0 10];
y0 = 1;
[t,y] = ode45(@simple_ode, tspan, y0);
plot(t, y);

function dydt = simple_ode(t, y)
dydt = t*sin(y);
end
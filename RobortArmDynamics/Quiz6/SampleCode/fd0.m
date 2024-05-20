function [dxdt] = fd0(t, x)
%fd forward dynamics of robot arm
%   called from ode45, input should by t and x
% Robot arm paraeters
m1 = 1.0; m2 = 1.0;
l1 = 1.0; l2 = 1.0; l1g = 0.5; l2g = 0.5;
d1 = 0.1; d2 = 0.1;
I1 = 0; I2 = 0;
dxdt = [x(3); x(4); -x(1); -x(2)];
end


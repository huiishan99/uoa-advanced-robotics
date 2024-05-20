function [dxdt] = fd3(t, x)
%fd forward dynamics of 2-link robot arm
%   large dumper coefficient
%   x(1) = th1;  x(2) = th2
%   x(3) = omg1; x(4) = omg2
%   tau = [0; 0]

% Gravity parameter 
g = 9.8;

% Robot arm paraeters
m1 = 1.0; m2 = 1.0;
l1 = 1.0; l2 = 1.0; lg1 = 0.5; lg2 = 0.5;
d1 = 1.0; d2 = 1.0;
I1 = 1/12 * m1 * l1.^2;
I2 = 1/12 * m2 * l2.^2;

% Intertia matrix
M = [I1 + I2 + m1*lg1.^2 + m2*l1.^2 + m2*lg2.^2 + 2*m2*l1*lg2+cos(x(2)),...
     I2 + m2*lg2.^2 + m2*l1*lg2*cos(x(2));...
     I2 + m2*lg2.^2 + m2*l1*lg2*cos(x(2)),...
     I2 + m2*lg2.^2 ];
H = [-m2*l1*lg2*(2*x(3)*x(4)+x(4).^2)*sin(x(2)) + d1*x(3);...
      m2*l1*lg2*x(3).^2*sin(x(2)) + d2*x(4)];
G = [m1*g*lg1*cos(x(1)) + m2*g*l1*cos(x(1)) + m2*g*lg2*cos(x(1)+x(2));...
     m2*g*lg2*cos(x(1)+x(2))];   

% Joint torque
Tau = [0; 0];

% Differential set equation
omg_d = inv(M)*(Tau - H - G);
dxdt = [x(3); x(4); omg_d(1); omg_d(2)];
end

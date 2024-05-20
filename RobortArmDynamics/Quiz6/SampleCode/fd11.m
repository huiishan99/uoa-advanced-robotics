function [dxdt] = fd11(t, x)
%fd forward dynamics of 2-link robot arm
%   Position PD control with medium gain
%   wtih gravity compensation
%   Target angle x(1) = pi/2; x(2) = 0;
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
%   Target angle x(1) = pi/2; x(2) = 0;
kp1 = 10; kp2 = 10;
kd1 = 5; kd2 = 5;
xd1 = pi/2; xd2 = 0;
Tau = [kp1*(xd1 - x(1)) + kd1*(-x(3)) + G(1);...
    kp2*(xd2 - x(2)) + kd2*(-x(4)) + G(2)];

% Differential set equation
omg_d = inv(M)*(Tau - H - G);
dxdt = [x(3); x(4); omg_d(1); omg_d(2)];
end

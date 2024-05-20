%
%   Dynamics 2link arm
%       Author: Keitaro Naruse
%       Date: 2021-05-18
%

% Solve differential equation of equations of robot motion
% fd0: small dumping coefficent d1, d2 = 0.01, no inertia
% [t,x] = ode45(@fd0, [0, 10],[0.1; 0.1; 0; 0]);
% fd1: small dumping coefficent d1, d2 = 0.01
% [t,x] = ode45(@fd1, [0, 10],[0.1; 0.1; 0; 0]);
% fd2: medium dumping coefficent  d1, d2 = 0.1
% [t,x] = ode45(@fd2, [0, 10],[0.1; 0.1; 0; 0]);
% fd3: large dumping coefficent  d1, d2 = 1
% [t,x] = ode45(@fd3, [0, 10],[0.1; 0.1; 0; 0]);
% fd4: Position feedback control with small gain
% [t,x] = ode45(@fd4, [0, 10],[0.1; 0.1; 0; 0]);
% fd5: Position feedback control with medium gain
% [t,x] = ode45(@fd5, [0, 10],[0.1; 0.1; 0; 0]);
% fd6: Position feedback control with large gain
% [t,x] = ode45(@fd6, [0, 10],[0.1; 0.1; 0; 0]);
% fd7: Position feedback control with small gain  plus gravity compensation
% [t,x] = ode45(@fd7, [0, 10],[0.1; 0.1; 0;  0]);
% fd8: Position feedback control with medium gain plus gravity compensation
% [t,x] = ode45(@fd8, [0, 10],[0.1; 0.1; 0; 0]);
% fd9: Position feedback control with large gain plus gravity compensation
% [t,x] = ode45(@fd9, [0, 10],[0.1; 0.1; 0; 0]);
% fd10: Position PD control with small gain  plus gravity compensation
% [t,x] = ode45(@fd10, [0, 30],[0.1; 0.1; 0;  0]);
% fd11: Position PD control with medium gain  plus gravity compensation
[t,x] = ode45(@fd11, [0, 10],[0.1; 0.1; 0;  0]);

% Plot th1 = x(1) and th(2) = x(2)
figure(1);
plot(t, x(:,1), 'r-', t, x(:,2), 'b-');
title('th1(red) and th2(blue)');

% Plot omg1 = x(4) and omg2 = x(2)
figure(2);
plot(t, x(:,3), 'r-', t, x(:,4), 'b-');
title('omg1(red) and omg2(blue)');

% Trajectory plot
l1 = 1.0; l2 = 1.0;
p0 = [0; 0];
p1 = zeros(2, length(t));
p2 = zeros(2, length(t));
figure(3);
for k=1:length(t)
    p1(:,k) = [l1*cos(x(k,1)); l1*sin(x(k,1))];
    p2(:,k) = p1(:,k)+[l2*cos(x(k,1)+x(k,2)); l2*sin(x(k,1)+x(k,2))];
    px = [p0(1), p1(1,k), p2(1,k)];
    py = [p0(2), p1(2,k), p2(2,k)];
    plot(px, py, 'b-o');
    axis equal;
    xlim([-2.5 2.5]);
    ylim([-2.5 2.5]);
    pause(1/100);
end

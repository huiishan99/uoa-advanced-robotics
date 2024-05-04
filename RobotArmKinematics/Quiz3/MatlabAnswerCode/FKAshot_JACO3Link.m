% Fixed joint angles for JACO arm
q1 = 0;  % Convert degrees to radians
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;

% DH Parameters
aa = (11.0 * pi) / 72.0;  % Angle for specific calculations
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.0743;
D5 = 0.0743;
D6 = 0.1687;
e2 = 0.0098;
d4b = D3 + sin(aa)/sin(2*aa) * D4;
d5b = sin(aa)/sin(2*aa) * (D4 + D5);
d6b = sin(aa)/sin(2*aa) * D5 + D6;

% Compute transformation matrices using DH parameters
T01 = DH(0, 0, D1, q1);
T12 = DH(-pi/2, 0, 0, q2);
T23 = DH(0, D2, e2, q3);
T34 = DH(-pi/2, 0, d4b, q4);
T45 = DH(aa, 0, d5b, q5);
T56 = DH(aa, 0, d6b, q6);

% Total transformation from base to each joint
T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;
T06 = T05 * T56;

% Extract positions for 2D and 3D plots
positions = [0, 0, 0; T01(1:3, 4)'; T02(1:3, 4)'; T03(1:3, 4)'; T04(1:3, 4)'; T05(1:3, 4)'; T06(1:3, 4)'];

% 2D plot
figure(1);
plot(positions(:, 1), positions(:, 2), '-o');
xlabel('X');
ylabel('Y');
title('2D View of JACO Arm');
pbaspect([1 1 1]);

% 3D plot
figure(2);
plot3(positions(:, 1), positions(:, 2), positions(:, 3), '-o');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D View of JACO Arm');
grid on;
pbaspect([1 1 1]);

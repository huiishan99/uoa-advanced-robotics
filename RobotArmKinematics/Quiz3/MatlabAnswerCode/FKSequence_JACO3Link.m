% Define joint angles (in radians)
q1 = linspace(0, pi/2, 91);
q2 = linspace(0, pi/2, 91);
q3 = linspace(0, pi/2, 91);
q4 = linspace(0, pi/2, 91);
q5 = linspace(0, pi/2, 91);
q6 = linspace(0, pi/2, 91);

% Define angle aa
aa = (11.0 * pi) / 72.0;

% According to your DH table parameters
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.0743;
D5 = 0.0743;
D6 = 0.1687;
e2 = 0.0098;

% Specially calculated d4b, d5b, d6b
d4b = D3 + sin(aa)/sin(2*aa) * D4;
d5b = sin(aa)/sin(2*aa) * (D4 + D5);
d6b = sin(aa)/sin(2*aa) * D5 + D6;

% Start plotting
figure(1); % 2D plot
hold on;
figure(2); % 3D plot
view(3);
hold on;

% Loop through calculations and plot
for i = 1:length(q1)
    % Using these parameters to construct transformation matrices
    T01 = DH(0, 0, D1, q1(i));
    T12 = DH(-pi/2, 0, 0, q2(i));
    T23 = DH(0, D2, e2, q3(i));
    T34 = DH(-pi/2, 0, d4b, q4(i));
    T45 = DH(aa, 0, d5b, q5(i));
    T56 = DH(aa, 0, d6b, q6(i));

    % Total transformation matrix
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    T06 = T05 * T56;

    % Extract position information for each joint
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
end

hold off;

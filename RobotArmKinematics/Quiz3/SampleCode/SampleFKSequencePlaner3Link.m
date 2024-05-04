% Link lengths
L1 = 1;
L2 = 1;
L3 = 1;

% Joint angles
q1 = [0:1:90];
q2 = [0:1:90];
q3 = [0:1:90];

% Draw in 2D, Loop for joint angles
figure(1);
hold on
for i = 1:91
    % Homegeneous transformation matricies for given angles
    % a frame to previous frame
    T01 = DH(0,  0, 0, deg2rad( q1(i)) );
    T12 = DH(0, L1, 0, deg2rad( q2(i)) );
    T23 = DH(0, L2, 0, deg2rad( q3(i)) );
    T34 = DH(0, L3, 0, deg2rad(  0   ) );

    % a frame to base 
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;

    % Points in base frame
    p00 = [0; 0; 0; 1];
    p01 = T01 * [0; 0; 0; 1];
    p02 = T02 * [0; 0; 0; 1];
    p03 = T03 * [0; 0; 0; 1];
    p04 = T04 * [0; 0; 0; 1];
    
    % Plot
    x = [p00(1), p01(1), p02(1), p03(1), p04(1)];
    y = [p00(2), p01(2), p02(2), p03(2), p04(2)];
    z = [p00(3), p01(3), p02(3), p03(3), p04(3)];
    plot(x, y, '-o');
    xlim([-3, 3]);
    ylim([-3, 3]);
    pbaspect([1 1 1]);
end
hold off

% Draw in 3D, Loop for joint angles
figure(2);
view(3);
hold on
for i = 1:91
    % Homegeneous transformation matricies for given angles
    % a frame to previous frame
    T01 = DH(0,  0, 0, deg2rad( q1(i)) );
    T12 = DH(0, L1, 0, deg2rad( q2(i)) );
    T23 = DH(0, L2, 0, deg2rad( q3(i)) );
    T34 = DH(0, L3, 0, deg2rad(  0   ) );

    % a frame to base 
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;

    % Points in base frame
    p00 = [0; 0; 0; 1];
    p01 = T01 * [0; 0; 0; 1];
    p02 = T02 * [0; 0; 0; 1];
    p03 = T03 * [0; 0; 0; 1];
    p04 = T04 * [0; 0; 0; 1];
    
    % Plot
    x = [p00(1), p01(1), p02(1), p03(1), p04(1)];
    y = [p00(2), p01(2), p02(2), p03(2), p04(2)];
    z = [p00(3), p01(3), p02(3), p03(3), p04(3)];
    plot3(x, y, z, '-o');
    pbaspect([1 1 1]);
end
hold off

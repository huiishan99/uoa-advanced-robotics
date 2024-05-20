function P = ForwardKinematics(Q)
    % Link lengths
    L1 = 1;
    L2 = 1;
    L3 = 1;

    % Homogeneous transformation matrices
    T01 = DH(0,  0, 0, Q(1));
    T12 = DH(pi/2, L1, 0, Q(2));
    T23 = DH(0, L2, 0, Q(3));
    T34 = DH(0, L3, 0, 0);

    % Combined transformation matrices
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;

    % Positions from the base to each joint
    p01 = T01 * [0; 0; 0; 1];
    p02 = T02 * [0; 0; 0; 1];
    p03 = T03 * [0; 0; 0; 1];
    p04 = T04 * [0; 0; 0; 1];

    P = [p01(1:3), p02(1:3), p03(1:3), p04(1:3)];
end

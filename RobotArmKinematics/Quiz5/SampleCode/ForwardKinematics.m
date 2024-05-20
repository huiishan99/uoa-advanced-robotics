function [P] = ForwardKinematics(Q)
%ForwardKinematics 
%   Forward kinematics of a hand tip
%   Input: Q = [q1; q2; q3], Joint angle vector
%   Output: P = [p1, p2, p3, p4], Arm pose vector
%             = [x1, x2, x3, x4]
%               [y1, y2, y3, y4]
%               [z1, z2, z3, z4]

% Link lengths
L1 = 1;
L2 = 1;
L3 = 1;

% Homegeneous transformation matricies for given angles
% a frame to previous frame
T01 = DH(0,  0, 0, Q(1));
T12 = DH(0, L1, 0, Q(2));
T23 = DH(0, L2, 0, Q(3));
T34 = DH(0, L3, 0, 0   );

% a frame to base 
T02 = T01 * T12;
T03 = T01 * T12 * T23;
T04 = T01 * T12 * T23 * T34;

% Points in base frame
p01 = T01 * [0; 0; 0; 1];
p02 = T02 * [0; 0; 0; 1];
p03 = T03 * [0; 0; 0; 1];
p04 = T04 * [0; 0; 0; 1];

P = [p01(1:3), p02(1:3), p03(1:3), p04(1:3)];
end

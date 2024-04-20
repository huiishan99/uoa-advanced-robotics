% Vehicle frame to world frame
Dx = 0.0; Dy = 0.0; Dz = 0.0;
R = deg2rad(90.0);
P = deg2rad(0.0);
Y = deg2rad(0.0);

% Define transformation matrices for each vehicle position
% Position 1 (no rotation)
T_V_W1 = HTTrans([2; 1; 0]);

% Position 2 (no rotation)
T_V_W2 = HTTrans([3; 1; 0]);

% Position 3 (rotation pi/2 around the z-axis)
T_V_W3 = HTTrans([5; 3; 0]) * HTRotZ(pi/2);

% Position 4 (rotation pi/2 around the z-axis)
T_V_W4 = HTTrans([5; 4; 0]) * HTRotZ(pi/2);

% Apply the transformations from vehicle to world frame for each position
pW = [T_V_W1*pC, T_V_W2*pC, T_V_W3*pC, T_V_W4*pC];

% Define the points in the camera frame
pC = [
     0.32, 0.32, -0.32, -0.32; % u = x
    -0.24, 0.24,  0.24, -0.24; % v = -y
     1.00, 1.00,  1.00,  1.0;  % z = depth
     1,    1,     1,     1     % constant
];

% Display points in vehicle and world frame at the same window
figure(1);
% Plot the original points
plot3(pC(1,:), pC(2,:), pC(3,:),'b+');
hold on;

% Plot the transformed points for each position
plot3(pW1(1,:), pW1(2,:), pW1(3,:), 'r*-'); 
plot3(pW2(1,:), pW2(2,:), pW2(3,:), 'r+-'); 
plot3(pW3(1,:), pW3(2,:), pW3(3,:), 'rx-'); 
plot3(pW4(1,:), pW4(2,:), pW4(3,:), 'ro-'); 

plot3([2, 3, 5, 5], [1, 1, 3, 4], [0, 0, 0, 0], 'k.-');

xlim([-1 7]); ylim([-1 7]); zlim([-1 7]);
grid on; pbaspect([1 1 1]);
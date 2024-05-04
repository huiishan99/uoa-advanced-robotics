%
%   Sample_T3D.m
%       Transfer a frame by 3D homogeneous transformation matrix
%       Author: Keitaro Naruse
%       Date: 2024-04-16 
%

% Vehicle frame to world frame
Dx = 0.0; Dy = 0.0; Dz = 0.0;
R = deg2rad(-90.0);
P = deg2rad( 0.0);
Y = deg2rad( 0.0);
% Transformation matrix from camera to world frame
T = HTTrans([Dx; Dy; Dz]) * HTRotZ(Y) * HTRotY(P) * HTRotX(R);

% Homegeneous points in camera frame 
pC = [
     0.32, 0.32, -0.32, -0.32; % u = x
    -0.24, 0.24,  0.24, -0.24; % v = -y
     1.00, 1.00,  1.00,  1.0;  % z = depth
     1,    1,     1,     1  % constant
    ];

% Homegeneous Points in world frame 
pW = [ T*pC(:,1), T*pC(:,2), T*pC(:,3), T*pC(:,4)];

% Display points in vehicle and world frame at the same window
figure(1);
plot3(pC(1,:), pC(2,:), pC(3,:),'b+-', pW(1,:), pW(2,:), pW(3,:),'r*-')
xlim([-3 3]); ylim([-3 3]); zlim([-3 3]);
grid on; pbaspect([1 1 1]);



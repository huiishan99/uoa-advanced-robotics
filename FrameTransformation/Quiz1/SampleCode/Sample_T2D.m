%
%   Sample_T2D.m
%       Transfer a frame by 2D homogeneous transformation matrix
%       Author: Keitaro Naruse
%       Date: 2022-04-12
%       MIT License
%

% Vehicle frame to world frame
X = 6.0; Y = 3.0; Q = deg2rad(60.0);
T = T2D(X, Y, Q);

% Homegeneous points in vehicle frame 
pV = [
     1,  1, -1, -1; % x
    -2,  4,  4, -2; % y
     1,  1,  1,  1  % constant
    ];

% Homegeneous Points in world frame 
pW = [ T*pV(:,1), T*pV(:,2), T*pV(:,3), T*pV(:,4)];

% Display points in vehicle and world frame at the same window
figure(1);
plot(pV(1,:), pV(2,:), 'b+-', pW(1,:), pW(2,:), 'r*-');
xlim([-10 10]); ylim([-10 10]); grid on; pbaspect([1 1 1]);

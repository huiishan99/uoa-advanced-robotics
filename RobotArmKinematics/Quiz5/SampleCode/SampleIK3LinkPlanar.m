% Sample-IK-3Link-Planar.m
%   Author: Keiaro Naruse
%   Date: 2021-04-20
%   License: MIT License

% Target position of hand tip
pD = [1; 1; 0];

% Iteration times
N = 50;

% Velocity coefficient
k = 0.1;

% Array of joint angle vectors
Q = zeros(3, N+1);

% Array of hand pose matrix
P = zeros(3, 4, N+1);

% Initial joint angles(rad)
Q(:,1) = [0.1; 0.2; 0.3];

% Forward kinematics is implemented in the function of ForwardKinematics
P(:,:,1) = ForwardKinematics(Q(:,1));

for i = 1:N
    % Jacobian
    J = NumericalJacobian(Q(:,i));
    % Inverse kinematics
    Q(:,i+1) = Q(:,i) + k * pinv(J) * (pD - P(:,4,i));
    % Forward kinematics is implemented in the function of ForwardKinematics
    P(:,:,i+1) = ForwardKinematics(Q(:,i+1));
end

% Plot a seqeunce of robot poses
figure(1);
for i = 1:1:N+1
    % Display as 3D plot
    x = P(1, :, i); y = P(2, :, i); z = P(3, :, i);
    figure(1);
    plot3(x, y, z, 'k-o');
    xlim([-3, 3]); ylim([-3, 3]); zlim([-3, 3]);
    pbaspect([1 1 1]);
    hold on;
end
% Plot a target position
plot3(pD(1), pD(2), pD(3), 'r*');

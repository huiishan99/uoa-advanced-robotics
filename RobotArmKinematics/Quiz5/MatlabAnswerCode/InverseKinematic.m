% InverseKinematics.m
% Inverse kinematics solution

% Target position
pD = [1; 1; 0];

% Iteration times and velocity coefficient
N = 50;
k = 0.1;

% Array of joint angle vectors and end-effector positions
Q = zeros(3, N+1);
P = zeros(3, 4, N+1);

% Initial joint angles
Q(:,1) = [0.1; 0.2; 0.3];

% Forward kinematics for the initial position
P(:,:,1) = ForwardKinematics(Q(:,1));

% Iteratively solve inverse kinematics
for i = 1:N
    J = NumericalJacobian(Q(:,i)); % Calculate Jacobian matrix
    Q(:,i+1) = Q(:,i) + k * pinv(J) * (pD - P(:,4,i)); % Inverse kinematics calculation
    P(:,:,i+1) = ForwardKinematics(Q(:,i+1)); % Update end-effector position
end

% Plot robot poses and target position
figure(1);
for i = 1:1:N+1
    x = P(1, :, i); y = P(2, :, i); z = P(3, :, i);
    plot3(x, y, z, 'k-o');
    xlim([-3, 3]); ylim([-3, 3]); zlim([-3, 3]);
    pbaspect([1 1 1]);
    hold on;
end
plot3(pD(1), pD(2), pD(3), 'r*');

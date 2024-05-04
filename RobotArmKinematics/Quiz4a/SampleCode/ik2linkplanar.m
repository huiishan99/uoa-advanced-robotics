% Link Length
L1 = 1.0;
L2 = 1.0;

% Simulation time
tMax = 100;

% Coefficient
a = 0.1;

% Goal of hand position
pd = [0; 1];

% Array of joint angle vectors
q = zeros(2, tMax+1);

% Array of elbow position vectors
p1 = zeros(2, tMax+1);

% Array of hand position vectors
p2 = zeros(2, tMax+1);

% Array of determinant
jd = zeros(1, tMax);

% Initial joint angle vector
q(:,1) = [deg2rad(10);deg2rad(20)];

% Initial hand position by forward kinematics
p1(:,1) = fk1( q(:,1), L1);
p2(:,1) = fk2( q(:,1), L1, L2 );

% Iteration
for t = 1:tMax
    % Jacobian
    J = jacob2(q(:,t), L1, L2);
    
    % Determinat
    jd(t) = det(J);
    
    % Inverse kinematics
    q(:,t+1) = q(:,t) + a * inv(J) * (pd - p2(:,t));
    % Forward kinematics
    p1(:,t+1) = fk1( q(:,t+1), L1);
    p2(:,t+1) = fk2( q(:,t+1), L1, L2 );
end

% Plot every 1 interval
figure(1)
hold on
for t = 1:1:tMax+1
    x = [0, p1(1, t), p2(1, t)];
    y = [0, p1(2, t), p2(2, t)];
    plot(x, y, '-o');
    xlim([-(L1+L2), (L1+L2)]);
    ylim([-(L1+L2), (L1+L2)]);
end
hold off
pbaspect([1 1 1]);

figure(2);
plot([1:tMax], jd);
xlim([0, tMax+1]);
ylim([-0.1, 1.1]);

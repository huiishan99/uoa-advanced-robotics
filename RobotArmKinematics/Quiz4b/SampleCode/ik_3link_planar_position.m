% Link Length setup
L1 = 1.0;
L2 = 1.0;
L3 = 1.0;

% Jacobian setup
syms th1 th2 th3
r = fk3([th1, th2, th3], L1, L2, L3);
jacob3 = jacobian(r, [th1, th2, th3]);

% Simulation time
tMax = 100;

% Coefficient
a = 0.1;

% Goal of hand position
pd = [0; 1];

% Array of joint angle vectors
q = zeros(3, tMax+1);
% Initial joint angle vector
q(:,1) = [deg2rad(10);deg2rad(20);deg2rad(30)];

% Array of elbow position vectors
p1 = zeros(2, tMax+1);
% Array of wrist position vectors
p2 = zeros(2, tMax+1);
% Array of hand position vectors
p3 = zeros(2, tMax+1);

% Initial hand position by forward kinematics
p1(:,1) = fk1( q(:,1), L1);
p2(:,1) = fk2( q(:,1), L1, L2 );
p3(:,1) = fk3( q(:,1), L1, L2, L3 );

% loop
for t = 1:tMax+1
    % Jacobian
    J = eval(subs(jacob3, [th1, th2, th3], [q(1,t), q(2,t), q(3,t)]));
    
    % Inverse kinematics
    q(:,t+1) = q(:,t) + a * pinv(J) * (pd - p3(:,t));
    
    % Forward kinematics
    p1(:,t+1) = fk1( q(:,t+1), L1);
    p2(:,t+1) = fk2( q(:,t+1), L1, L2 );
    p3(:,t+1) = fk3( q(:,t+1), L1, L2,L3 );
end

% Plot every 1 interval
figure(1)
hold on
for t = 1:1:tMax
    x = [0, p1(1, t), p2(1, t), p3(1, t)];
    y = [0, p1(2, t), p2(2, t), p3(2, t)];
    plot(x, y, '-o');
    xlim([-(L1+L2+L3), (L1+L2+L3)]);
    ylim([-(L1+L2+L3), (L1+L2+L3)]);
end
hold off
pbaspect([1 1 1]);

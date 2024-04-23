% Link Length setup
L1 = 0.5;
L2 = 0.5;
L3 = 0.5;

% Simulation time
tMax = 100;

% Coefficient
a = 0.1;

% Goal of hand position
pd = [0; 0.5];

% Array of joint angle vectors
q = zeros(3, tMax+1);
% Initial joint angle vectors for two scenarios
initial_angles = [deg2rad(10) deg2rad(10) deg2rad(10); 0 0 0];

% Iteration for two initial conditions
for ic = 1:2
    q(:,1) = initial_angles(ic, :)';

    % Array of elbow, wrist, and hand position vectors
    p1 = zeros(2, tMax+1);
    p2 = zeros(2, tMax+1);
    p3 = zeros(2, tMax+1);

    % Initial hand position by forward kinematics
    p1(:,1) = fk1(q(:,1), L1);
    p2(:,1) = fk2(q(:,1), L1, L2);
    p3(:,1) = fk3(q(:,1), L1, L2, L3);

    % loop
    for t = 1:tMax
        % Compute Jacobian numerically
        J = numericJacobian(q(:,t), L1, L2, L3);

        % Check if the Jacobian is near singular using the smallest singular value
        singular_values = svd(J);
        if min(singular_values) < 1e-6  % Threshold for numerical singularity
            fprintf('Jacobian is near singular at iteration %d with a small singular value of %f.\n', t, min(singular_values));
            break;
        end

        % Inverse kinematics
        q(:,t+1) = q(:,t) + a * pinv(J) * (pd - p3(:,t));

        % Forward kinematics
        p1(:,t+1) = fk1(q(:,t+1), L1);
        p2(:,t+1) = fk2(q(:,t+1), L1, L2);
        p3(:,t+1) = fk3(q(:,t+1), L1, L2, L3);
    end


    % Plot every 1 interval
    figure(ic);
    hold on;
    for t = 1:tMax
        x = [0, p1(1, t), p2(1, t), p3(1, t)];
        y = [0, p1(2, t), p2(2, t), p3(2, t)];
        plot(x, y, '-o');
        xlim([-(L1+L2+L3), (L1+L2+L3)]);
        ylim([-(L1+L2+L3), (L1+L2+L3)]);
    end
    hold off;
    pbaspect([1 1 1]);
end

function J = numericJacobian(q, L1, L2, L3)
    delta = 1e-5;
    J = zeros(2, 3);
    for i = 1:3
        dq = zeros(3,1);
        dq(i) = delta;
        p_plus = fk3(q + dq, L1, L2, L3);
        p_minus = fk3(q - dq, L1, L2, L3);
        J(:, i) = (p_plus - p_minus) / (2 * delta);
    end
end

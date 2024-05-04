% Link Lengths
L1 = 0.5; % Length of the first link
L2 = 0.5; % Length of the second link

% Simulation time
tMax = 100; % Total number of iterations for the simulation

% Coefficient for inverse kinematics update
a = 0.1; % Learning rate for the IK algorithm

% Goal of hand position
pd = [0; 0.5]; % Desired position of the end-effector

% Arrays for storing data for both initial conditions
q_initial_conditions = [[deg2rad(10); deg2rad(10)], [0; 0]]; % Initial joint angles in radians

% Process each set of initial conditions
for cond = 1:size(q_initial_conditions, 2)
    % Initialize arrays for joint angles, elbow positions, hand positions, and Jacobian determinants
    q = zeros(2, tMax+1); % Joint angles
    q(:,1) = q_initial_conditions(:, cond); % Set initial joint angles
    p1 = zeros(2, tMax+1); % Elbow positions
    p2 = zeros(2, tMax+1); % Hand positions
    jd = zeros(1, tMax); % Jacobian determinants

    % Calculate initial hand position using forward kinematics
    p1(:,1) = fk1(q(:,1), L1); % Position of elbow
    p2(:,1) = fk2(q(:,1), L1, L2); % Position of hand

    % Run the iterative IK algorithm
    for t = 1:tMax
        J = jacob2(q(:,t), L1, L2); % Compute the Jacobian at the current joint angles
        jd(t) = det(J); % Calculate the determinant of the Jacobian

        % Check for singularity by looking at the determinant
        if abs(jd(t)) < 1e-6
            fprintf('Singularity occurred at t=%d with determinant nearly zero. Stopping iteration.\n', t);
            break; % Exit the loop if a singularity is encountered
        end

        % Inverse kinematics step
        q(:,t+1) = q(:,t) + a * inv(J) * (pd - p2(:,t)); % Update joint angles

        % Update positions using forward kinematics
        p1(:,t+1) = fk1(q(:,t+1), L1); % Update elbow position
        p2(:,t+1) = fk2(q(:,t+1), L1, L2); % Update hand position
    end

    % Plotting the movement for the current set of initial conditions
    figure(cond);
    hold on;
    for t = 1:1:tMax+1
        plot([0, p1(1, t), p2(1, t)], [0, p1(2, t), p2(2, t)], '-o');
        % Setting the axis limits to fit the movement of the arm
        xlim([-(L1+L2), (L1+L2)]);
        ylim([-(L1+L2), (L1+L2)]);
    end
    hold off;
    pbaspect([1 1 1]); % Keep aspect ratio of the plot square
end

% Script to display robot arm pose
tspan = [0 10];
initial_conditions = [0.1; 0.1; 0; 0];

% List of control function handles
control_functions = {@fd0, @fd1, @fd2, @fd3, @fd4, @fd5, @fd6, @fd7, @fd8, @fd9, @fd10, @fd11};

% Iterate through each control function and display poses
for i = 1:length(control_functions)
    control_function = control_functions{i};
    [tx, x] = ode45(control_function, tspan, initial_conditions);
    
    % Display the robot arm pose
    figure;
    for j = 1:length(tx)
        armpose(x(j, 1), x(j, 2));
    end
end

function armpose(th1, th2)
    % Robot arm parameters
    l1 = 1.0;  % Length of the first link
    l2 = 1.0;  % Length of the second link
    
    % Calculate joint positions
    x0 = 0; y0 = 0;
    x1 = l1 * cos(th1);
    y1 = l1 * sin(th1);
    x2 = x1 + l2 * cos(th1 + th2);
    y2 = y1 + l2 * sin(th1 + th2);
    
    % Plot the robot arm
    plot([x0 x1], [y0 y1], 'r-o');
    hold on;
    plot([x1 x2], [y1 y2], 'b-o');
    hold off;
    axis equal;
    axis([-2 2 -2 2]);
    drawnow;
end

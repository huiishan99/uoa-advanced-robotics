% Main script: Run all control functions and plot results
tspan = [0 10];
initial_conditions = [0.1; 0.1; 0; 0];

% List of control function handles
control_functions = {@fd0, @fd1, @fd2, @fd3, @fd4, @fd5, @fd6, @fd7, @fd8, @fd9, @fd10, @fd11};

% Iterate through each control function and plot results
for i = 1:length(control_functions)
    control_function = control_functions{i};
    [tx, x] = ode45(control_function, tspan, initial_conditions);
    
    % Create a new figure window
    figure;
    
    % Plot th1 and th2
    subplot(2,1,1);
    plot(tx, x(:,1), 'r-', tx, x(:,2), 'b-');
    title(['th1 (red) and th2 (blue) for fd', num2str(i-1)]);
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('th1', 'th2');
    
    % Plot omg1 and omg2
    subplot(2,1,2);
    plot(tx, x(:,3), 'r-', tx, x(:,4), 'b-');
    title(['omg1 (red) and omg2 (blue) for fd', num2str(i-1)]);
    xlabel('Time (s)');
    ylabel('Angular velocity (rad/s)');
    legend('omg1', 'omg2');
end

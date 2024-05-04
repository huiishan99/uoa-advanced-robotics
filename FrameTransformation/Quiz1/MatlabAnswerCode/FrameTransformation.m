% Define local points in homogenous coordinates
pV = [
    0.8, -0.8, -0.8, 0.8;
    0.8, 0.8, -0.8, -0.8;
    1, 1, 1, 1
    ];

% Define transformations as [x; y; q]
transformations = [
    2, 1, 0;
    3, 1, 0;
    4, 1, 0;
    4, 2, pi/2;
    4, 3, pi/2
];

% Initialize a figure
figure; hold on;
plot(pV(1,:), pV(2,:), 'b+');
xlim([-10, 10]); ylim([-10, 10]); grid on; pbaspect([1 1 1]);

% Defines the poins style
styles = {'r*', 'r+', 'ro', 'rx', 'r.'};

% Apply each transformation and plot points
for i = 1:size(transformations, 1)
    T = T2D(transformations(i,1), transformations(i,2), transformations(i,3));
    pW = T * pV; % Transform points to world frame
    
    % Plot points
    plot(pW(1,:), pW(2,:), styles{i});
end


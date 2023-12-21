%% Plot real position v estimate

%% Load data
fileExcel = 'test.csv';

dataFileOptitrack = 'test.mat';

dataExcel = readtable(fileExcel);
load(dataFileOptitrack);

%% Optitrack, Real position
% Remove .mat and {}
loadedVariables = who('-file', dataFileOptitrack);
loadedVariableName = loadedVariables{1}; 

% Rename to data
dataOptitrack = eval(loadedVariableName);

X = squeeze(dataOptitrack.Trajectories.Labeled.Data(:, 1, :))';
Y = squeeze(dataOptitrack.Trajectories.Labeled.Data(:, 2, :))';


%% GoLang Excel, Estimate position

x_t = dataExcel(:, end-2); %  x is the third last value in first collum
y_t = dataExcel(:, end-1); %  y is the second last value in first collum
theta_t = dataExcel(:, end); %  theta is the last value in first collum

% For plotting, from table to double
x = table2array(x_t) * 10; 
y = table2array(y_t) * 10;
theta = table2array(theta_t); 



%% Plot real position v estimate

% Plot real position
figure;
% plot(X(:), Y(:), 'b-', 'LineWidth', 2); % Blue lines for real position
hold on;

% Plot estimate position
plot(x, y, 'r-', 'LineWidth', 2); % Red lines for estimated position

% Set square size
squareSize = 1000;

squareCorners = [
    0, 0;              
    squareSize, 0;
    squareSize, squareSize;
    0, squareSize;
];

% Add markers at corners of the square
scatter(squareCorners(:, 1), squareCorners(:, 2), 100, 'g', 'Marker', 'o', 'LineWidth', 2);
% Blue marker at (0, 0)
scatter(0, 0, 100, 'g', 'Marker', 'o', 'LineWidth', 2); 

% labels and title
xlabel('X Position');
ylabel('Y Position');
title('Real Position vs. Estimated Position');

% legend
% legend('Real Position', 'Estimated Position');
legend('Estimated Position');

% legend('Real position', 'Estimated position', 'Label 3', 'Location', 'Best');
% legend_position = [x, y, width, height]; 
% set(legend, 'Position', legend_position);


% grid
grid on;

% axis equal;
axis ([-100 1100 -100 1100]);
%




%
% limit space on the sides
% xlim([min(min(X(:)), min(x)) - 100, max(max(X(:)), max(x)) + 100]);

hold off;


% Save as PNG in current directory, check directory with cmd 'pwd'
saveas(gcf, 'fikant_test_1_odometry.png'); 





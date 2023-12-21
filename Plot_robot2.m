%% Plotting for square test in drone lab A0052
% Access the X and Y data within the struct
% "Your test  name".Trajectories.Labeled.Data



%% Specify your data file name:  

dataFile = 'test.mat';

%% Data handling 
load(dataFile);

% Remove .mat and {}
loadedVariables = who('-file', dataFile);
loadedVariableName = loadedVariables{1}; 

% Rename to data
data = eval(loadedVariableName);

X = data.Trajectories.Labeled.Data(:, 1, :);
Y = data.Trajectories.Labeled.Data(:, 2, :);

% Reshape data 2D
X = squeeze(X)';
Y = squeeze(Y)';



% Create a 2D plot of X and Y
figure;
plot(X, Y, 'LineWidth', 1, 'Color', 'b');

%% Markers
hold on; %  'hold on' to plot on the same figure
% plot(500, 500, 'ro', 'MarkerSize', 8, 'LineWidth', 1); 
% plot(0, 500, 'ro', 'MarkerSize', 8, 'LineWidth', 1); 
% plot(500, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 1); 
plot(0, 0, 'go', 'MarkerSize', 8, 'LineWidth', 1); 


% Markers for 1m (1000, 1000) test
plot(1000, 1000, 'ro', 'MarkerSize', 8, 'LineWidth', 1); 
plot(0, 1000, 'ro', 'MarkerSize', 8, 'LineWidth', 1); 
plot(1000, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 1); 

%%

% plot(min(X), min(Y), 'go', 'MarkerSize', 8, 'LineWidth', 1); % 'ro' specifies a red circle as a marker
% axis and grid
% axis equal;

% Make space at edges
%  axis([-50 600 -150 550]); % for square test

% axis([-150 650 -150 650]);

% axis([-100 1100 -100 1100]); % for 1m square test

% Legend
legend('Real Position');
legend('Location', 'southeast');

xlim([min(min(X(:)), min(x)) - 100, max(max(X(:)), max(x)) + 100]);


grid on;

% Add labels and a title
xlabel('X Position');
ylabel('Y Position');
title('2D robot movement');

% Save as PNG in current directory, check directory with cmd 'pwd'
% saveas(gcf, 'firkant_test_1_opti.png'); 



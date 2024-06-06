%% Plot 'x' and 'y' from excel file GoLang server 

%% To save or not to save as png
% true - file saves to specified directory
% false - file is not saved

saveFileTheta = true;
saveFilePosition = true;

%% Data
% Specify the Excel file name and sheet name
file = 'init_test_2_CCW.csv';

% extract file name to use in saving as PNG
[~, basename, ~] = fileparts(file);

% Read
data = readtable(file);

% Extract
[rows, collums] = size(data);

x = table2array(data(:, end-2)); %  x is the third last value in first collum
y = table2array(data(:, end-1)); %  y is the second last value in first collum

theta_degrees = table2array(data(:, end)); %  theta is the last value in first collum
theta_radians = deg2rad(theta_degrees);

t = table2array(data(:, 1));
tSeconds = seconds(t - t(1)); % Convert 't' to seconds from the first timestamp


%% Plots

%% Plot 1: 2D Position

figure; 
plot(x, y, '-', 'Color', 'r', 'LineWidth', 1);
hold on;
xlabel('X');
ylabel('Y');
title('2D position of robot based on odometry data, CWW');

% axis auto;                % Automatically adjust axis 
axis ([-10 110 -10 110]);   % Set scaling manually if auto does not look ideal. 
grid;

% Specific points as black circles 
plot(0, 0,      'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'none', 'LineWidth', 1.5);  
plot(100, 0,    'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'none', 'LineWidth', 1.5);
plot(0, 100,    'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'none', 'LineWidth', 1.5);  
plot(100, 100,  'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'none', 'LineWidth', 1.5);

%% Save 2D Position as PNG


if saveFilePosition
    folder = 'C:\Users\denek\OneDrive\Skrivebord\master pictures\folder';
    if ~exist(folder, 'dir')
        mkdir(folder);
    end
    saveas(gcf, [folder '\' basename '_2D_position.png']);
end

% saveas(gcf, [basename '_2D_position.png']);



%% Plot 2: Theta Angle

figure; % Create a new figure window for theta plot
plot(tSeconds, theta_degrees, '-', 'Color', 'r', 'LineWidth', 1); % Plot theta over time
hold on;
xlabel('Time (seconds)');
ylabel('Theta (degrees)');
title('Theta angle over time, CWW');

axis auto;  % Automatically adjust axis limits
grid;


% ADD LINES AT 0 90 180 270 degrees
yline(0, 'k--', 'LineWidth', 0.5);
yline(90, 'k--', 'LineWidth', 0.5);
yline(180, 'k--', 'LineWidth', 0.5);
yline(270, 'k--', 'LineWidth', 0.5);
yline(-90, 'k--', 'LineWidth', 0.5);





%% Save Theta Angle as PNG

% saveas(gcf, [basename '_Theta_angle_over_time.png']); 

if saveFileTheta
    % Save in specified directory
    folder = 'C:\path\plots';
    if ~exist(folder, 'dir')
        mkdir(folder);
    end
    saveas(gcf, [folder '\' basename '_Theta_angle.png']);
end






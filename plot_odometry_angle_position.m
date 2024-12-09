%% Plot odometry data from GoLang server 

% To save or not to save the plots as PNGs
saveFileTheta = true; % Change to false if you don't want to save
saveFilePosition = true; % Change to false if you don't want to save

% Specify the CSV file path directly
odometry_data_file = 'odometry_data/positions_D_24_10_T_17_31.csv'; % Odometry data file

% Define the folder to save the plots in
plotsFolder = 'odometry_data_plots';

%% Load data and run functions
% Read the data from the CSV file
data = readtable(odometry_data_file);

% Extract the basename for saving plot files
[~, basename, ~] = fileparts(odometry_data_file);

% Extract relevant columns
x = table2array(data(:, end-2)); % x is the third last column
y = table2array(data(:, end-1)); % y is the second last column
theta_degrees = table2array(data(:, end)); % theta is the last column
theta_radians = deg2rad(theta_degrees); % Convert to radians if needed

% Extract and process time data
t = table2array(data(:, 1));
tSeconds = seconds(t - t(1)); % Convert to seconds from the first timestamp

plotPosition(x, y, saveFilePosition, basename, plotsFolder);
plotTheta(tSeconds, theta_degrees, saveFileTheta, basename, plotsFolder);

%% Function to plot 2D Position
function plotPosition(x, y, saveFilePosition, basename, plotsFolder)
    % Convert input data from cm to mm
    x = x * 10;
    y = y * 10;
    
    % Determine axis limits with margins
    margin = 0.1; % Margin as a percentage of the data range
    xRange = max(x) - min(x);
    yRange = max(y) - min(y);
    
    xMin = min(x) - margin * xRange;
    xMax = max(x) + margin * xRange;
    yMin = min(y) - margin * yRange;
    yMax = max(y) + margin * yRange;
    
    figure; 
    plot(x, y, '-', 'Color', 'r', 'LineWidth', 1);
    hold on;
    xlabel('X [mm]');
    ylabel('Y [mm]');
    title('2D position of robot based on odometry data');
    
    % Set axis limits with margins
    axis([xMin, xMax, yMin, yMax]);
    
    grid on;
    
    % Plot specific reference points
    plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'none', 'LineWidth', 1.5);  
    
    % Save 2D Position plot if required
    if saveFilePosition
        saveas(gcf, fullfile(plotsFolder, [basename '_2D_position.png']));
    end
end

%% Function to plot Theta over time
function plotTheta(tSeconds, theta_degrees, saveFileTheta, basename, plotsFolder)
    figure; 
    plot(tSeconds, theta_degrees, '-', 'Color', 'r', 'LineWidth', 1); % Plot theta over time
    hold on;
    xlabel('Time (seconds)');
    ylabel('Theta (degrees)');
    title('Robot heading (Theta) over time');
    
    axis auto;  % Automatically adjust axis limits
    grid on;
    
    % Add horizontal reference lines for 0, 90, 180, and 270 degrees
    yline(0, 'k--', 'LineWidth', 0.5);
    yline(90, 'k--', 'LineWidth', 0.5);
    yline(180, 'k--', 'LineWidth', 0.5);
    yline(270, 'k--', 'LineWidth', 0.5);
    yline(-90, 'k--', 'LineWidth', 0.5);
    
    % Save Theta plot if required
    if saveFileTheta
        saveas(gcf, fullfile(plotsFolder, [basename '_Theta_angle.png']));
    end
end

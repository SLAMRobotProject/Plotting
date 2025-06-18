%% Plot odometry data 

odometry_data_file = 'data/odometry_data/may_25/positions_lyapunov/positions_pid_lyap_turn_v1_1.csv'; 
plotsFolder = 'odometry_plots/positions_pid_lyap_turn_v1';

if ~isfolder(plotsFolder)
    mkdir(plotsFolder);
end

try
    data = readtable(odometry_data_file);
catch ME
    error("Error reading the file: %s\nEnsure the file path is correct.", ME.message);
end

[~, basename, ~] = fileparts(odometry_data_file);

x = table2array(data(:, 3));      
y = table2array(data(:, 4));       
theta_degrees = table2array(data(:, 5)); 
theta_radians = deg2rad(theta_degrees);  

t = table2array(data(:, 1));
tSeconds = seconds(t - t(1));


plotPosition(x, y, basename, plotsFolder);
plotTheta(tSeconds, theta_degrees, basename, plotsFolder);

%% Function to plot 2D Position
function plotPosition(x, y, basename, plotsFolder)
    x = x * 10;
    y = y * 10;
    
    margin = 0.1; 
    xRange = max(x) - min(x);
    yRange = max(y) - min(y);
    
    xMin = min(x) - margin * xRange;
    xMax = max(x) + margin * xRange;
    yMin = min(y) - margin * yRange;
    yMax = max(y) + margin * yRange;

    figure; 
    plot(x, y, '-', 'Color', 'r', 'LineWidth', 1, 'DisplayName', 'Robot Path');
    hold on;
    xlabel('X [mm]');
    ylabel('Y [mm]');
    title('2D position of robot based on odometry data');
    
    axis([xMin, xMax, yMin, yMax]);
    
    grid on;

    plot(0, 0, 'bo', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'Origin (0, 0)');  

    legend('show', 'Location', 'best');

    saveas(gcf, fullfile(plotsFolder, [basename '_2D_position.png']));
    close;
end

%% Function to plot Theta over time
function plotTheta(tSeconds, theta_degrees, basename, plotsFolder)
    figure; 
    plot(tSeconds, theta_degrees, '-', 'Color', 'r', 'LineWidth', 1, 'DisplayName', 'Theta (degrees)'); % Plot theta over time
    hold on;
    xlabel('Time (seconds)');
    ylabel('Theta (degrees)');
    title('Robot heading (Theta) over time');

    axis auto;  
    xlim([0 100]);
    grid on;

    refAngles = [0, 90, 180, 270, -90];
    for angle = refAngles
        yline(angle, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    end

    legend('show', 'Location', 'best');

    saveas(gcf, fullfile(plotsFolder, [basename '_Theta_angle.png']));
    close;
end

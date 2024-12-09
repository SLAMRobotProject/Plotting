%% Plotting for square test in drone lab A0052

clear;
clc;

% plotAllMatFiles('optitrack_data/', 'optitrack_plots/', 'tower');
% plotDataFromOptitrackMatFile('optitrack_data/slamrobot_dk5_test2_01_10_combined.mat', 'optitrack_plots/', 'tower');

plotOptitrackAndOdometry('optitrack_data/slamrobot_dk5_test2_01_10_combined.mat', ...
                     'odometry_data/positions_D_24_10_T_17_31.csv', ...
                     'optitrack_plots/', ...
                     'tower');


%% Plot OptiTrack data single .mat file
function plotDataFromOptitrackMatFile(dataFile, outputFolder, desired_label)
    % plotDataFromMatFile loads a .mat file containing trajectory data, filters the data 
    % based on the provided label (default = tower), and generates a 2D plot of the robot movement.
    % The start of the trajectory is moved to the origin (0,0) and the
    % first edge is aligned with the first axis the robot moves along.
    %
    % Input:
    % - dataFile:     The full path to the .mat file containing the trajectory data
    % - outputFolder: The folder where the plot will be saved
    % - label:        (Optional) A label used to filter the trajectory data. 
    %                 If empty, default tower trajectories will be plotted.
    %
    % The plot includes:
    % - A blue line for the trajectory
    % - A green circle marking the starting point
    % - A red square representing the track boundary

    % Check if the label argument is provided; if not, set it to 'tower'
    if nargin < 3
        desired_label = 'tower';  % Set to 'tower' if no label is provided
    end

    % disp(['Desired label: "', desired_label, '" given as argument']);
    
    % Load data from the specified file
    load(dataFile);
    
    % Get the loaded variable name and assign it to 'data'
    loadedVariables = who('-file', dataFile);
    loadedVariableName = loadedVariables{1};  
    data = eval(loadedVariableName);
    
    % Extract position labels and data
    position_labels = data.Trajectories.Labeled.Labels;
    
    % Check if label is valid
    if ~ismember(desired_label, position_labels)
        % If the desired label is not in position_labels, throw an error
        error(['Invalid label "', desired_label, '". The available labels are: ', strjoin(position_labels, ', ')]);
    end

    X = data.Trajectories.Labeled.Data(:, 1, :);
    Y = data.Trajectories.Labeled.Data(:, 2, :);
    
    % Reshape the data to 2D (trajectories in X and Y directions)
    X = squeeze(X)';
    Y = squeeze(Y)';

    % Loop over each trajectory and process only those matching the desired label
    for pos_idx = 1:length(position_labels)
        label = position_labels{pos_idx};
        
        % Check if the label matches the desired label
        if ~isempty(desired_label) && ~strcmp(label, desired_label)
            continue;  % Skip trajectories that don't match the desired label
        end
   
        % Extract X and Y for the current trajectory
        X = X(:, pos_idx);
        Y = Y(:, pos_idx);

        % Adjust the trajectory so that the first point is at origin (0,0)
        X = X - X(1);
        Y = Y - Y(1);

        min_distance = 150;
        rotationAngle = calculateInitialRotationAngle(X, Y, min_distance); 
        % disp(['Rotation angle: ', num2str(rotationAngle * 180 / pi), ' degrees']);
        
        % Calculate the rotation matrix based on the calculated angle
        rotationMatrix = [cos(rotationAngle), -sin(rotationAngle);
                          sin(rotationAngle), cos(rotationAngle)];
        
        % Concatenate X and Y into a 2xN matrix (where N is the number of points)
        coords = [X'; Y'];
        
        % Perform matrix multiplication with the rotation matrix
        rotatedCoords = (rotationMatrix * coords)';
            
        % Extract the rotated X and Y coordinates
        X = rotatedCoords(:, 1);
        Y = rotatedCoords(:, 2);
    
        % Create a 2D plot of X and Y
        figure;
        hold on;
    
        % Plot the trajectory (in blue)
        hTrajectory = plot(X, Y, 'LineWidth', 1, 'Color', 'b');  
        
        % Mark the starting point with a green circle based on the first data point
        hStartPoint = plot(X(1), Y(1), 'go', 'MarkerSize', 8, 'LineWidth', 1); 
        
        % Draw a red square for the track boundary (corners at (0,0), (1000,0), (1000,1000), (0,1000))
        hSquare = plot([0 1000 1000 0 0], [0 0 1000 1000 0], 'r-', 'LineWidth', 1);
        
        % Set axis limits and grid, ensuring equal scaling for X and Y axes
        axis equal;
        xlim([-500 1500]);
        ylim([-500 1500]);
        grid on;
    
        % Add labels and title
        xlabel('X Position [mm]');
        ylabel('Y Position [mm]');
        title(['2D robot movement ', label]);
    
        % Combine legends
        legend([hStartPoint, hSquare, hTrajectory(1)], ...
               {'Start Point (Green Circle)', 'Track Boundary (Red Square)', 'Real Position (Blue)'}, ...
               'Location', 'northeast');
    
        % Automatically generate a filename based on the input data file
        [~, baseFileName, ~] = fileparts(dataFile);
        
        % Append label to filename if provided
        if ~isempty(label)
            saveFileName = fullfile(outputFolder, strcat(baseFileName, '_', label, '_plot.png'));
        else
            saveFileName = fullfile(outputFolder, strcat(baseFileName, '_plot.png'));
        end
    
        % Save the plot as a PNG
        saveas(gcf, saveFileName);
    
        % Close the figure to avoid too many open figure windows
        close(gcf);
    end
end 

%% Plot OptiTrack data all .mat files in a folder

function plotAllMatFiles(inputFolder, outputFolder, label)
    % plotAllMatFiles processes all .mat files in the specified input folder, generates 2D plots
    % of robot movement trajectories, and saves them as PNG files in the specified output folder.
    %
    % Input:
    % - inputFolder:   The folder containing the .mat files with trajectory data
    % - outputFolder:  The folder where the plot images will be saved
    % - label:         (Optional) A specific label to filter the trajectories for plotting. 
    %                  If not provided or empty, all trajectories will be plotted.
    %
    % Usage:
    % - plotAllMatFiles('data/', 'plots/')
    %   This will process all .mat files in 'data/' and save the generated plots in 'plots/'.
    %
    % - plotAllMatFiles('data/', 'plots/', 'tower')
    %   This will process all .mat files in 'data/', but only plot the trajectories labeled 'tower'.
    
    % Check if the label argument is provided; if not, set it to empty
    if nargin < 3
        label = '';  % Set to empty if no label is provided
    end
    
    % Create the output folder if it doesn't exist
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end
    
    % Get list of all .mat files in the input folder
    matFiles = dir(fullfile(inputFolder, '*.mat'));
    
    % Loop over each .mat file in the folder
    for k = 1:length(matFiles)
        % Get the full path of the .mat file
        matFileName = fullfile(inputFolder, matFiles(k).name);
        
        % Load and plot the data
        plotDataFromMatFile(matFileName, outputFolder, label);
    end
end

%% Plot OptiTrack and Odometry data
function plotOptitrackAndOdometry(dataFile, odometryFile, outputFolder, desired_label)
    % plotOptitrackAndOdometry generates a 2D plot of trajectory data (real position) from
    % a .mat file and overlays it with odometry data (estimated position) from a .csv file.
    % The plot includes the trajectory, odometry data, and other relevant markers.
    %
    % Input:
    % - dataFile: The full path to the .mat file containing trajectory data.
    % - odometryFile: The full path to the .csv file containing odometry data.
    % - outputFolder: The folder where the plot will be saved.
    % - desired_label: (Optional) A label used to filter the trajectory data.

    if nargin < 4
        desired_label = 'tower'; % Default label
    end

    % Load trajectory data
    load(dataFile);
    loadedVariables = who('-file', dataFile);
    loadedVariableName = loadedVariables{1};
    data = eval(loadedVariableName);

    % Extract position labels and data
    position_labels = data.Trajectories.Labeled.Labels;

    if ~ismember(desired_label, position_labels)
        error(['Invalid label "', desired_label, '". The available labels are: ', strjoin(position_labels, ', ')]);
    end

    X = data.Trajectories.Labeled.Data(:, 1, :);
    Y = data.Trajectories.Labeled.Data(:, 2, :);
    X = squeeze(X)';
    Y = squeeze(Y)';

    % Load odometry data
    odometryData = readtable(odometryFile);
    odom_X = table2array(odometryData(:, end-2)) * 10; % Convert to mm
    odom_Y = table2array(odometryData(:, end-1)) * 10;

    % disp('First odometry coordinate: X = %.2f mm, Y = %.2f mm\n', odom_X(1), odom_Y(1));

    for pos_idx = 1:length(position_labels)
        label = position_labels{pos_idx};

        if ~isempty(desired_label) && ~strcmp(label, desired_label)
            continue;
        end

        % Extract X and Y for the current trajectory
        traj_X = X(:, pos_idx);
        traj_Y = Y(:, pos_idx);

        % Adjust trajectory to start at origin
        traj_X = traj_X - traj_X(1);
        traj_Y = traj_Y - traj_Y(1);

        % Calculate rotation angle and rotate trajectory
        min_distance = 150;
        rotationAngle = calculateInitialRotationAngle(traj_X, traj_Y, min_distance);
        rotationMatrix = [cos(rotationAngle), -sin(rotationAngle); sin(rotationAngle), cos(rotationAngle)];
        coords = [traj_X'; traj_Y'];
        rotatedCoords = (rotationMatrix * coords)';
        traj_X = rotatedCoords(:, 1);
        traj_Y = rotatedCoords(:, 2);

        % Create the plot
        figure;
        hold on;

        % Plot real trajectory
        hTrajectory = plot(traj_X, traj_Y, 'LineWidth', 1, 'Color', 'b');

        % Plot odometry data
        hOdometry = plot(odom_X, odom_Y, 'LineWidth', 1, 'Color', 'm');

        % Mark starting point
        hStartPoint = plot(traj_X(1), traj_Y(1), 'go', 'MarkerSize', 8, 'LineWidth', 1);

        % Plot track boundary
        hSquare = plot([0 1000 1000 0 0], [0 0 1000 1000 0], 'r-', 'LineWidth', 1);

        % Configure plot
        axis equal;
        xlim([-500 1500]);
        ylim([-500 1500]);
        grid on;
        xlabel('X Position [mm]');
        ylabel('Y Position [mm]');
        title(['2D Robot Movement Optitrack and Odometry: ', label]);

        % Add legend
        legend([hStartPoint, hSquare, hTrajectory(1), hOdometry(1)], ...
               {'Start Point (Green Circle)', 'Track Boundary (Red Square)', ...
                'Real Position (Blue)', 'Odometry Data (Red)'}, ...
               'Location', 'northeast');

        % Save plot
        [~, baseFileName, ~] = fileparts(dataFile);
        saveFileName = fullfile(outputFolder, strcat(baseFileName, '_', label, '_with_odometry_plot.png'));
        saveas(gcf, saveFileName);

        % Close figure
        close(gcf);
    end
end

%% Calculate the initial rotation angle
function rotationAngle = calculateInitialRotationAngle(X, Y, min_distance)
    % calculateInitialRotationAngle detects the first corner of the robot's trajectory, 
    % and calculates the angle between the first segment and the X-axis.
    
    % Initialize variables
    num_points = length(X);  % Number of points in the trajectory
    corners = [];  % Store corner points (endpoints of segments)
    rotationAngle = 0;  % Default to no rotation if no valid segments are found
    
    % Compute the absolute differences for X and Y coordinates
    diffX = abs(diff(X));
    diffY = abs(diff(Y));

    % Calculate dynamic threshold based on variability
    varianceX = var(diffX(1:min(1000, num_points - 1)));
    varianceY = var(diffY(1:min(1000, num_points - 1)));
    
    % Set dynamic threshold as a factor of the variance
    threshold = 0.5 * sqrt(varianceX + varianceY);  % Example scaling factor

    % Determine initial changing coordinate
    if mean(diffX(1:min(1000, num_points - 1))) > mean(diffY(1:min(1000, num_points - 1)))
        changing_coord = 'X';
    else
        changing_coord = 'Y';
    end

    % Start detecting segments
    start_idx = 1;
    for i = 2:num_points-1
        if strcmp(changing_coord, 'X') && abs(diffX(i)) < threshold
            % Change detected, calculate and record distance
            x_start = X(start_idx);
            y_start = Y(start_idx);
            x_end = X(i);
            y_end = Y(i);

            % Euclidean distance
            segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

            if segment_distance >= min_distance
                corners = [corners; x_start, y_start];  % Store start point as corner
                % disp(['Corner found at: (' num2str(x_start) ', ' num2str(y_start) ')']);
                start_idx = i + 1;  % Update start index for next segment
            end

            changing_coord = 'Y';  % Switch changing coordinate
        elseif strcmp(changing_coord, 'Y') && abs(diffY(i)) < threshold
            % Change detected, calculate and record distance
            x_start = X(start_idx);
            y_start = Y(start_idx);
            x_end = X(i);
            y_end = Y(i);

            % Euclidean distance
            segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

            if segment_distance >= min_distance
                corners = [corners; x_start, y_start];  % Store start point as corner
                % disp(['Corner found at: (' num2str(x_start) ', ' num2str(y_start) ')']);
                start_idx = i + 1;  % Update start index for next segment
            end

            changing_coord = 'X';  % Switch changing coordinate
        end
    end

    % Catch the last segment if missed and ensure the last corner is added
    if start_idx < num_points
        x_start = X(start_idx);
        y_start = Y(start_idx);
        x_end = X(end);
        y_end = Y(end);

        segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

        if segment_distance >= min_distance
            corners = [corners; x_end, y_end];  % Store final end point as corner
            % disp(['Corner found at: (' num2str(x_end) ', ' num2str(y_end) ')']);
        end
    end

    % Ensure the last corner is added if it's missing
    if isempty(corners) || (corners(end, 1) ~= X(end) || corners(end, 2) ~= Y(end))
        corners = [corners; X(end), Y(end)];  % Add last point as corner
        % disp(['Corner found at: (' num2str(X(end)) ', ' num2str(Y(end)) ')']);
    end

    % If there are at least two corners, calculate the rotation angle based on the first segment
    if size(corners, 1) > 1
        % Calculate the vector from the first corner to the second corner
        dx = corners(2, 1) - corners(1, 1);
        dy = corners(2, 2) - corners(1, 2);
            
        rotationAngle = atan2(dy, dx);  % Calculate angle
        
        % Adjust rotation direction based on the sign of the non-primary coordinate
        if (abs(dx) > abs(dy) && dy < 0) || (abs(dy) > abs(dx) && dx < 0)
            rotationAngle = -rotationAngle;  % Reverse the rotation direction
        end
    end
end


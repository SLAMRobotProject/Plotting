%% Example Usage

plotOptitrackAndOdometry('data\OptiTrack_data\may_25\nrf6_PID_v1\nrf6_PID_v1_10.mat', ...
                      'data\odometry_data\may_25\positions_PID\positions_pid_v1_10.csv', ...
                      'optitrack_plots/', ...
                      'tower');

plotAllMatFiles('data\OptiTrack_data\may_25\nrf6_lqr_kinematic\', 'optitrack_plots/lqr_kinematic', 'tower');

%% Plotting for square test in drone lab A0052

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
    % - plotAllMatFiles('data/', 'plots/', 'tower')

    if nargin < 3
        label = '';
    end

    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end

    matFiles = dir(fullfile(inputFolder, '*.mat'));

    for k = 1:length(matFiles)
        matFileName = fullfile(inputFolder, matFiles(k).name);
        plotDataFromOptitrackMatFile(matFileName, outputFolder, label);
    end
end

function plotDataFromOptitrackMatFile(dataFile, outputFolder, desired_label)
    % plotDataFromMatFile loads a .mat file containing trajectory data, filters the data 
    % based on the provided label (default = tower), and generates a 2D plot of the robot movement.

    if nargin < 3
        desired_label = 'tower';
    end

    disp(['Loading data from file: ', dataFile]);
    load(dataFile);

    loadedVariables = who('-file', dataFile);
    loadedVariableName = loadedVariables{1};
    data = eval(loadedVariableName);

    position_labels = data.Trajectories.Labeled.Labels;

    if ~ismember(desired_label, position_labels)
        error(['Invalid label "', desired_label, '". The available labels are: ', strjoin(position_labels, ', ')]);
    end

    X = squeeze(data.Trajectories.Labeled.Data(:, 1, :))';
    Y = squeeze(data.Trajectories.Labeled.Data(:, 2, :))';

    for pos_idx = 1:length(position_labels)
        label = position_labels{pos_idx};
        if ~isempty(desired_label) && ~strcmp(label, desired_label)
            continue;
        end

        if length(position_labels) > 1
            X_curr = X(:, pos_idx);
            Y_curr = Y(:, pos_idx);
        else
            X_curr = X';
            Y_curr = Y';
        end

        X_curr = X_curr - X_curr(1);
        Y_curr = Y_curr - Y_curr(1);

        min_distance = 150;
        rotationAngle = calculateInitialRotationAngle(X_curr, Y_curr, min_distance);

        rotationMatrix = [cos(rotationAngle), -sin(rotationAngle); sin(rotationAngle), cos(rotationAngle)];
        coords = [X_curr'; Y_curr'];
        rotatedCoords = (rotationMatrix * coords)';

        X_rot = rotatedCoords(:, 1);
        Y_rot = rotatedCoords(:, 2);

        figure;
        hold on;
        hTrajectory = plot(X_rot, Y_rot, 'LineWidth', 1, 'Color', 'b');
        hStartPoint = plot(X_rot(1), Y_rot(1), 'go', 'MarkerSize', 8, 'LineWidth', 1);
        hSquare = plot([0 1000 1000 0 0], [0 0 1000 1000 0], 'r-', 'LineWidth', 1);

        axis equal;
        xlim([-500 1500]);
        ylim([-500 1500]);
        grid on;

        xlabel('X Position [mm]');
        ylabel('Y Position [mm]');
        title(['2D Robot Movement Optitrack: ', label]);

        legend([hStartPoint, hSquare, hTrajectory(1)], ...
               {'Start Point (Green Circle)', 'Square track (Red)', 'Real Position (Blue)'}, ...
               'Location', 'northeast');

        [~, baseFileName, ~] = fileparts(dataFile);
        if ~isempty(label)
            saveFileName = fullfile(outputFolder, strcat(baseFileName, '_', label, '_plot.png'));
        else
            saveFileName = fullfile(outputFolder, strcat(baseFileName, '_plot.png'));
        end

        disp(['Saving plot as: ', saveFileName]);
        saveas(gcf, saveFileName);
        close(gcf);
    end
end

function rotationAngle = calculateInitialRotationAngle(X, Y, min_distance)
    % calculateInitialRotationAngle detects the first corner of the robot's trajectory, 
    % and calculates the angle between the first segment and the X-axis.

    num_points = length(X);
    corners = [];
    rotationAngle = 0;

    diffX = abs(diff(X));
    diffY = abs(diff(Y));
    varianceX = var(diffX(1:min(1000, num_points - 1)));
    varianceY = var(diffY(1:min(1000, num_points - 1)));
    threshold = 0.5 * sqrt(varianceX + varianceY);

    if mean(diffX(1:min(1000, num_points - 1))) > mean(diffY(1:min(1000, num_points - 1)))
        changing_coord = 'X';
    else
        changing_coord = 'Y';
    end

    start_idx = 1;
    for i = 2:num_points-1
        if strcmp(changing_coord, 'X') && abs(diffX(i)) < threshold
            x_start = X(start_idx); y_start = Y(start_idx);
            x_end = X(i); y_end = Y(i);
            segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

            if segment_distance >= min_distance
                corners = [corners; x_start, y_start];
                start_idx = i + 1;
            end
            changing_coord = 'Y';

        elseif strcmp(changing_coord, 'Y') && abs(diffY(i)) < threshold
            x_start = X(start_idx); y_start = Y(start_idx);
            x_end = X(i); y_end = Y(i);
            segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

            if segment_distance >= min_distance
                corners = [corners; x_start, y_start];
                start_idx = i + 1;
            end
            changing_coord = 'X';
        end
    end

    if start_idx < num_points
        x_start = X(start_idx); y_start = Y(start_idx);
        x_end = X(end); y_end = Y(end);
        segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

        if segment_distance >= min_distance
            corners = [corners; x_end, y_end];
        end
    end

    if isempty(corners) || (corners(end, 1) ~= X(end) || corners(end, 2) ~= Y(end))
        corners = [corners; X(end), Y(end)];
    end

    if size(corners, 1) > 1
        dx = corners(2, 1) - corners(1, 1);
        dy = corners(2, 2) - corners(1, 2);
        rotationAngle = atan2(dy, dx);

        if (abs(dx) > abs(dy) && dy < 0) || (abs(dy) > abs(dx) && dx < 0)
            rotationAngle = -rotationAngle;
        end
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
    %
    % Usage:
    % plotOptitrackAndOdometry('optitrack_data.mat', 'odometry.csv', 'plots/', 'tower');

    if nargin < 4
        desired_label = 'tower'; 
    end

    disp(['Loading data from file: ', dataFile]);
    load(dataFile);
    loadedVariables = who('-file', dataFile);
    loadedVariableName = loadedVariables{1};
    data = eval(loadedVariableName);

    position_labels = data.Trajectories.Labeled.Labels;

    if ~ismember(desired_label, position_labels)
        error(['Invalid label "', desired_label, '". The available labels are: ', strjoin(position_labels, ', ')]);
    end

    X = data.Trajectories.Labeled.Data(:, 1, :);
    Y = data.Trajectories.Labeled.Data(:, 2, :);
    X = squeeze(X)';
    Y = squeeze(Y)';

    odometryData = readtable(odometryFile);
    odom_X = table2array(odometryData(:, 3)) * 10; % Convert to mm
    odom_Y = table2array(odometryData(:, 4)) * 10;

    rotationMatrix = [0, 1; -1, 0];  
    odom_coords = [odom_X'; odom_Y'];  
    rotatedOdometry = (rotationMatrix * odom_coords)'; 
    odom_X_rot = rotatedOdometry(:, 1);  
    odom_Y_rot = rotatedOdometry(:, 2); 

    for pos_idx = 1:length(position_labels)
        label = position_labels{pos_idx};

        if ~isempty(desired_label) && ~strcmp(label, desired_label)
            continue;
        end

        if length(position_labels) > 1
            traj_X = X(:, pos_idx);
            traj_Y = Y(:, pos_idx);
        else
            traj_X = X'; 
            traj_Y = Y';  
        end

        traj_X = traj_X - traj_X(1);
        traj_Y = traj_Y - traj_Y(1);

        min_distance = 150;
        rotationAngle = calculateInitialRotationAngle(traj_X, traj_Y, min_distance);
        rotationMatrix = [cos(rotationAngle), -sin(rotationAngle); sin(rotationAngle), cos(rotationAngle)];
        coords = [traj_X'; traj_Y'];
        rotatedCoords = (rotationMatrix * coords)';
        traj_X = rotatedCoords(:, 1);
        traj_Y = rotatedCoords(:, 2);

        figure;
        hold on;

        hTrajectory = plot(traj_X, traj_Y, 'LineWidth', 1, 'Color', 'b');

        hOdometry = plot(odom_X_rot, odom_Y_rot, 'LineWidth', 1, 'Color', 'm');

        hStartPoint = plot(traj_X(1), traj_Y(1), 'go', 'MarkerSize', 8, 'LineWidth', 1);

        % hSquare = plot([0 1000 1000 0 0], [0 0 1000 1000 0], 'r-', 'LineWidth', 1);

        axis equal;
        xlim([-500 1500]);
        ylim([-500 1500]);
        grid on;
        xlabel('X Position [mm]');
        ylabel('Y Position [mm]');
        title(['2D Robot Movement Optitrack and Odometry: ', label]);

        legend([hStartPoint, hTrajectory(1), hOdometry(1)], ...
               {'Start Point (Green Circle)', ...
                'Real Position (Blue)', 'Odometry Data (Magenta)'}, ...
               'Location', 'northeast');

        % legend([hStartPoint, hSquare, hTrajectory(1), hOdometry(1)], ...
        %       {'Start Point (Green Circle)', 'Square track (Red)', ...
        %        'Real Position (Blue)', 'Odometry Data (magenta)'}, ...
        %       'Location', 'northeast');

        [~, baseFileName, ~] = fileparts(dataFile);
        saveFileName = fullfile(outputFolder, strcat(baseFileName, '_', label, '_with_odometry_plot.png'));
        disp(['Saving plot as: ', saveFileName]);
        
        saveas(gcf, saveFileName);
        close(gcf);
    end
end

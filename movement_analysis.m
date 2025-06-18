%% Analyse single file:

clear; 
clc;

dataFile = 'data\OptiTrack_data\may_25\nrf6_lqr_dynamic_integral_v2\nrf6_lqr_dynamic_integral_v2_1.mat';

load(dataFile);

loadedVariables = who('-file', dataFile);
loadedVariableName = loadedVariables{1}; 

data = eval(loadedVariableName);

X = data.Trajectories.Labeled.Data(:, 1, :);
Y = data.Trajectories.Labeled.Data(:, 2, :);
X = squeeze(X);  % Convert the X data into a 2D matrix
Y = squeeze(Y);  % Convert the Y data into a 2D matrix

position_labels = data.Trajectories.Labeled.Labels;

% Define a minimum distance for segments to be considered valid (e.g., 800 mm)
min_distance = 700;
label = "tower";

[segment_distances, segment_labels, corners, angles] = Movement_Analysis_Single_File(X, Y, min_distance, position_labels, label);

% Print segment distances for the 'tower' label
for i = 1:length(segment_distances)
    fprintf('Distance for segment %d "tower": %.2f mm\n', i, segment_distances(i));
end

% Print the calculated angles
for i = 1:length(angles)
    fprintf('Angle at corner %d: %.2f degrees\n', i, angles(i));
end

% Specify the turn number that corresponds to the 180-degree turn (e.g., 4)
turn_number_for_180 = 4;

[average_distance, std_distance, average_angles_90, std_angles_90, average_angles_180, std_angles_180] = CalculateAveragesOneFile(segment_distances, angles, turn_number_for_180);

% Display the results
fprintf('Average Segment Distance: %.2f mm (std: %.2f mm)\n', average_distance, std_distance);
fprintf('Average of 90-degree turns: %.2f degrees (std: %.2f degrees)\n', average_angles_90, std_angles_90);
fprintf('Average of 180-degree turn: %.2f degrees (std: %.2f degrees)\n', average_angles_180, std_angles_180);

%% Analyse all files in a folder

clear;
clc;

% Folder containing all the .mat files
folderPath = 'data\OptiTrack_data\may_25\nrf6_pid_lyap_turn_v1';


% Define a minimum distance for segments to be considered valid (e.g., 800 mm)
min_distance = 700;
label = 'tower';  
% Specify the turn number that corresponds to the 180-degree turn (e.g., 4)
turn_number_for_180 = 4;

[final_average_distance, final_std_distance, final_average_90, final_std_90, final_average_180, final_std_180] = AnalyzeFolder(folderPath, min_distance, label, turn_number_for_180);

% Display the results
fprintf('Final Average Segment Distance: %.2f mm (std: %.2f mm)\n', final_average_distance, final_std_distance);
fprintf('Final Average of 90-degree turns: %.2f degrees (std: %.2f degrees)\n', final_average_90, final_std_90);
fprintf('Final Average of 180-degree turn: %.2f degrees (std: %.2f degrees)\n', final_average_180, final_std_180);


function [segment_distances, segment_labels, corners, angles] = Movement_Analysis_Single_File(X, Y, min_distance, position_labels, desired_label)
    % Movement_Analysis_Single_File analyzes the trajectory of the robot for the "tower" position label only,
    % detects corners, and calculates angles between segments.
    % Input:
    % X, Y            - Trajectories of the robot in X and Y directions
    % min_distance    - Minimum segment distance threshold
    % position_labels - Labels for each sensor/position (e.g., 'back right', 'tower')
    % Output:
    % segment_distances - The distances traveled along each valid segment of the path
    % segment_labels    - Labels indicating which measurement each segment came from
    % corners           - Coordinates of detected corners (i.e., segment endpoints)
    % angles            - Angles between consecutive segments

    if size(X, 1) < size(X, 2)
        X = X';
    end
    if size(Y, 1) < size(Y, 2)
        Y = Y';
    end

    num_points = size(X, 1);
    num_positions = size(X, 2);
    segment_distances = [];
    segment_labels = {};
    corners = [];
    angles = [];

    for pos_idx = 1:num_positions
        label = position_labels{pos_idx};
        if ~strcmp(label, desired_label)
            continue;
        end

        x_data = X(:, pos_idx);
        y_data = Y(:, pos_idx);

        diffX = abs(diff(x_data));
        diffY = abs(diff(y_data));

        varianceX = var(diffX(1:1000));
        varianceY = var(diffY(1:1000));
        threshold = 0.6 * sqrt(varianceX + varianceY);

        if mean(diffX(1:1000)) > mean(diffY(1:1000))
            changing_coord = 'X';
        else
            changing_coord = 'Y';
        end

        start_idx = 1;
        corner_points = [];

        for i = 2:num_points-1
            if  (strcmp(changing_coord, 'X') && abs(diffX(i)) < threshold)
                x_start = x_data(start_idx);
                y_start = y_data(start_idx);
                x_end = x_data(i);
                y_end = y_data(i);
                segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

                if segment_distance >= min_distance
                    segment_distances = [segment_distances, segment_distance];
                    segment_labels{end+1} = label;
                    corner_points = [corner_points; x_start, y_start];
                    start_idx = i + 1;
                end
                changing_coord = 'Y';

            elseif (strcmp(changing_coord, 'Y') && abs(diffY(i)) < threshold)
                x_start = x_data(start_idx);
                y_start = y_data(start_idx);
                x_end = x_data(i);
                y_end = y_data(i);
                segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

                if segment_distance >= min_distance
                    segment_distances = [segment_distances, segment_distance];
                    segment_labels{end+1} = label;
                    corner_points = [corner_points; x_start, y_start];
                    start_idx = i + 1;
                end
                changing_coord = 'X';
            end
        end

        if start_idx < num_points
            x_start = x_data(start_idx);
            y_start = y_data(start_idx);
            x_end = x_data(end);
            y_end = y_data(end);
            segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

            if segment_distance >= min_distance
                segment_distances = [segment_distances, segment_distance];
                segment_labels{end+1} = label;
                corner_points = [corner_points; x_end, y_end];
            end
        end

        valid_idx = find(~isnan(x_data) & ~isnan(y_data), 1, 'last');
        if ~isempty(valid_idx)
            x_final = x_data(valid_idx);
            y_final = y_data(valid_idx);
            if isempty(corner_points) || norm(corner_points(end,:) - [x_final, y_final]) > 1e-3
                corner_points = [corner_points; x_final, y_final];
            end
        end

        corners = [corners; corner_points];

        if size(corner_points, 1) > 2
            angles = [angles, calculate_segment_angles(corner_points)];
        end
    end
end

function angles = calculate_segment_angles(corner_points)
    % calculate_segment_angles calculates the angles between consecutive segments based on corner points.
    % Inputs:
    %   corner_points - A Nx2 matrix where each row contains the (x, y) coordinates of a corner point.
    %                   At least 3 corner points are required to calculate angles.
    % Outputs:
    %   angles - A row vector containing the calculated angles (in degrees) between consecutive segments.
    %            The number of angles returned will be (N-2), where N is the number of corner points.

    num_corners = size(corner_points, 1);
    angles = [];

    for i = 2:num_corners-1
        v1 = corner_points(i, :) - corner_points(i-1, :);
        v2 = corner_points(i+1, :) - corner_points(i, :);
        angle = atan2d(norm(det([v1; v2])), dot(v1, v2));
        angles = [angles, angle];
    end
end

function [average_distance, std_distance, average_angles_90, std_angles_90, average_angles_180, std_angles_180] = CalculateAveragesOneFile(all_distances, all_angles, turn_number_for_180)
    % CalculateAverages calculates the average and standard deviation of segment distances, average 90-degree turns, and 180-degree turn
    % Inputs:
    %   all_distances   - An array where each element contains the segment distances from one test
    %   all_angles      - An array where each element contains the angles from one test
    %   turn_number_for_180 - The turn number (e.g., 4) that is expected to be 180 degrees
    % Outputs:
    %   average_distance - The average segment distance across all tests
    %   std_distance     - The standard deviation of segment distances
    %   average_angles_90 - The average angles for 90-degree turns across all tests
    %   std_angles_90    - The standard deviation of the 90-degree turns
    %   average_angles_180 - The average angle for the 180-degree turn
    %   std_angles_180   - The standard deviation of the 180-degree turn

    average_distance = mean(all_distances);
    std_distance = std(all_distances, 1);

    angles_90 = [];
    angles_180 = [];

    for turn_idx = 1:length(all_angles)
        if turn_idx == turn_number_for_180
            angles_180 = [angles_180, all_angles(turn_idx)];
        else
            angles_90 = [angles_90, all_angles(turn_idx)];
        end
    end

    average_angles_90 = mean(angles_90);
    std_angles_90 = std(angles_90, 1);
    average_angles_180 = mean(angles_180);
    std_angles_180 = std(angles_180, 1);
end

function [final_average_distance, final_std_distance, final_average_90, final_std_90, final_average_180, final_std_180] = AnalyzeFolder(folderPath, min_distance, label, turn_number_for_180)
    % AnalyzeFolder processes all .mat files in the folder, runs Movement_Analysis_Single_File on each file,
    % and calculates the average and standard deviation of segment distances, 90-degree turns, and 180-degree turns.
    % Inputs:
    %   folderPath - Path to the folder containing the .mat files
    %   min_distance - Minimum segment distance threshold
    %   label - The label of the position to analyze (e.g., 'tower')
    %   turn_number_for_180 - The turn number that corresponds to the 180-degree turn (e.g., 4)
    % Outputs:
    %   final_average_distance - The overall average segment distance across all files
    %   final_std_distance - The overall standard deviation of segment distances
    %   final_average_90 - The overall average angle for 90-degree turns
    %   final_std_90 - The overall standard deviation of 90-degree turns
    %   final_average_180 - The overall average angle for the 180-degree turn
    %   final_std_180 - The overall standard deviation of the 180-degree turn

    files = dir(fullfile(folderPath, '*.mat'));

    all_distances = [];
    all_angles_90 = [];
    all_angles_180 = [];

    for file_idx = 1:length(files)
        dataFile = fullfile(files(file_idx).folder, files(file_idx).name);
        load(dataFile);
        loadedVariables = who('-file', dataFile);
        loadedVariableName = loadedVariables{1};
        data = eval(loadedVariableName);

        X = data.Trajectories.Labeled.Data(:, 1, :);
        Y = data.Trajectories.Labeled.Data(:, 2, :);
        X = squeeze(X)';
        Y = squeeze(Y)';

        position_labels = data.Trajectories.Labeled.Labels;

        [segment_distances, ~, ~, angles] = Movement_Analysis_Single_File(X, Y, min_distance, position_labels, label);

        valid_distances = segment_distances(segment_distances >= 800);

        angles_90 = [];
        angles_180 = [];
        for turn_idx = 1:length(angles)
            if turn_idx == turn_number_for_180
                angles_180 = [angles_180, angles(turn_idx)];
            else
                angles_90 = [angles_90, angles(turn_idx)];
            end
        end

        all_distances = [all_distances, valid_distances];
        all_angles_90 = [all_angles_90, angles_90];
        all_angles_180 = [all_angles_180, angles_180];
    end

    if isempty(all_distances)
        final_average_distance = NaN;
        final_std_distance = NaN;
        warning('No valid distances found.');
    else
        final_average_distance = mean(all_distances);
        final_std_distance = std(all_distances);
    end

    if isempty(all_angles_90)
        final_average_90 = NaN;
        final_std_90 = NaN;
        warning('No valid 90-degree angles found.');
    else
        final_average_90 = mean(all_angles_90);
        final_std_90 = std(all_angles_90);
    end

    if isempty(all_angles_180)
        final_average_180 = NaN;
        final_std_180 = NaN;
        warning('No valid 180-degree angles found.');
    else
        final_average_180 = mean(all_angles_180);
        final_std_180 = std(all_angles_180);
    end
end

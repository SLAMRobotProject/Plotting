%% Analyse single file:

clear; 
clc;

dataFile = 'optitrack_data/slamrobot_dk6_test1_01_10_combined_slowStart.mat';

% Load your data
load(dataFile);

% Remove .mat and {}
loadedVariables = who('-file', dataFile);
loadedVariableName = loadedVariables{1}; 

% Rename to data
data = eval(loadedVariableName);

% Extract trajectory data
X = data.Trajectories.Labeled.Data(:, 1, :);
Y = data.Trajectories.Labeled.Data(:, 2, :);
X = squeeze(X)';  % Convert the X data into a 2D matrix
Y = squeeze(Y)';  % Convert the Y data into a 2D matrix

% Extract the labels directly from the dataset
position_labels = data.Trajectories.Labeled.Labels;

% Define a minimum distance for segments to be considered valid (e.g., 10 mm)
min_distance = 100;
label = "tower";

% Analyze the movement segments for each labeled trajectory
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

% Call the function to calculate the averages and population standard deviations
[average_distance, std_distance, average_angles_90, std_angles_90, average_angles_180, std_angles_180] = CalculateAveragesOneFile(segment_distances, angles, turn_number_for_180);

% Display the results
fprintf('Average Segment Distance: %.2f mm (std: %.2f mm)\n', average_distance, std_distance);
fprintf('Average of 90-degree turns: %.2f degrees (std: %.2f degrees)\n', average_angles_90, std_angles_90);
fprintf('Average of 180-degree turn: %.2f degrees (std: %.2f degrees)\n', average_angles_180, std_angles_180);

%% Analyse multiple files in a folder
clear;
clc;

% Folder containing all the .mat files
folderPath = 'optitrack_test_folder\';  % Specify the folder path

% Define parameters for the analysis
min_distance = 100;  % Example minimum segment distance threshold
label = 'tower';  % The label to analyze (e.g., 'tower')
turn_number_for_180 = 4;  % The turn number that corresponds to the 180-degree turn

% Call the function to analyze all files and compute the overall averages and population standard deviations
[final_average_distance, final_std_distance, final_average_90, final_std_90, final_average_180, final_std_180] = AnalyzeFolder(folderPath, min_distance, label, turn_number_for_180);

% Display the results
fprintf('Final Average Segment Distance: %.2f mm (std: %.2f mm)\n', final_average_distance, final_std_distance);
fprintf('Final Average of 90-degree turns: %.2f degrees (std: %.2f degrees)\n', final_average_90, final_std_90);
fprintf('Final Average of 180-degree turn: %.2f degrees (std: %.2f degrees)\n', final_average_180, final_std_180);


%% Movement analysis functions

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
    
    % Initialize variables
    num_points = size(X, 1);  % Number of points in each trajectory
    num_positions = size(X, 2);  % Number of measurement positions (columns)
    segment_distances = [];  % Array to store distances of all segments
    segment_labels = {};     % Cell array to store corresponding labels
    corners = [];            % Store corner points (endpoints of segments)
    angles = [];             % Store the angles between consecutive segments

    for pos_idx = 1:num_positions
        % Only process data if the label is 'tower'
        label = position_labels{pos_idx};
        if ~strcmp(label, desired_label)
            continue;  % Skip measurements that are not 'tower'
        end
        
        % Extract data for current 'tower' position (one trajectory)
        x_data = X(:, pos_idx);
        y_data = Y(:, pos_idx);
        
        % Compute the absolute differences for X and Y coordinates
        diffX = abs(diff(x_data));
        diffY = abs(diff(y_data));

        % Calculate dynamic threshold based on variability
        varianceX = var(diffX(1:1000));
        varianceY = var(diffY(1:1000));
        
        % Set dynamic threshold as a factor of the variance
        threshold = 0.5 * sqrt(varianceX + varianceY);  % Example scaling factor

        % Determine initial changing coordinate
        if mean(diffX(1:1000)) > mean(diffY(1:1000))
            changing_coord = 'X';
        else
            changing_coord = 'Y';
        end

        % Start detecting segments
        start_idx = 1;
        corner_points = [];  % Temporary storage for corner points in this trajectory
        
        for i = 2:num_points-1
            if strcmp(changing_coord, 'X') && abs(diffX(i)) < threshold
                % Change detected, calculate and record distance
                x_start = x_data(start_idx);
                y_start = y_data(start_idx);
                x_end = x_data(i);
                y_end = y_data(i);

                % Euclidean distance
                segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

                if segment_distance >= min_distance
                    segment_distances = [segment_distances, segment_distance];
                    segment_labels{end+1} = label;
                    corner_points = [corner_points; x_start, y_start];  % Store start point as corner
                    
                    % Update start index for next segment
                    start_idx = i + 1;
                end

                % Switch changing coordinate
                changing_coord = 'Y';
            elseif strcmp(changing_coord, 'Y') && abs(diffY(i)) < threshold
                % Change detected, calculate and record distance
                x_start = x_data(start_idx);
                y_start = y_data(start_idx);
                x_end = x_data(i);
                y_end = y_data(i);

                % Euclidean distance
                segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

                if segment_distance >= min_distance
                    segment_distances = [segment_distances, segment_distance];
                    segment_labels{end+1} = label;
                    corner_points = [corner_points; x_start, y_start];  % Store start point as corner
                    
                    % Update start index for next segment
                    start_idx = i + 1;
                end

                % Switch changing coordinate
                changing_coord = 'X';
            end
        end

        % Catch the last segment if missed and ensure the last corner is added
        if start_idx < num_points
            x_start = x_data(start_idx);
            y_start = y_data(start_idx);
            x_end = x_data(end);
            y_end = y_data(end);

            segment_distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);

            if segment_distance >= min_distance
                segment_distances = [segment_distances, segment_distance];
                segment_labels{end+1} = label;
                corner_points = [corner_points; x_end, y_end];  % Store final end point as corner
            end
        end
        
        % Ensure the last corner is added if it's missing
        if isempty(corner_points) || (corner_points(end, 1) ~= x_data(end) || corner_points(end, 2) ~= y_data(end))
            corner_points = [corner_points; x_data(end), y_data(end)];  % Add last point as corner
        end
        
        % Add corner points of this trajectory to the global corner list
        corners = [corners; corner_points];
        
        % Calculate angles between consecutive segments
        if size(corner_points, 1) > 2
            angles = [angles, calculate_segment_angles(corner_points)];
        end
    end
end

% Function to calculate angles between consecutive segments
function angles = calculate_segment_angles(corner_points)
    % calculate_segment_angles calculates the angles between consecutive segments based on corner points.
    % Inputs:
    %   corner_points - A Nx2 matrix where each row contains the (x, y) coordinates of a corner point.
    %                   At least 3 corner points are required to calculate angles.
    % Outputs:
    %   angles - A row vector containing the calculated angles (in degrees) between consecutive segments.
    %            The number of angles returned will be (N-2), where N is the number of corner points.
    
    % Get the number of corner points
    num_corners = size(corner_points, 1);
    
    % Initialize the array to store the calculated angles
    angles = [];
    
    % Loop through the corner points to calculate the angles between consecutive segments
    for i = 2:num_corners-1
        % Define vectors for two consecutive segments:
        % v1 is the vector from corner i-1 to i
        % v2 is the vector from corner i to i+1
        v1 = corner_points(i, :) - corner_points(i-1, :);
        v2 = corner_points(i+1, :) - corner_points(i, :);
        
        % Calculate the angle between v1 and v2 using the cross and dot products
        angle = atan2d(norm(det([v1; v2])), dot(v1, v2));  % Result in degrees
        
        % Store the calculated angle in the angles array
        angles = [angles, angle];
    end
end

% Function to calculate the averages and standard deviations of the segments and angles
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

    % Calculate the average and standard deviation of all segment distances
    average_distance = mean(all_distances);
    std_distance = std(all_distances, 1); % Add this line for standard deviation
    
    % Initialize variables to calculate averages for 90-degree and 180-degree turns
    angles_90 = [];
    angles_180 = [];

    % Classify angles into 90-degree or 180-degree categories
    for turn_idx = 1:length(all_angles)
        if turn_idx == turn_number_for_180
            % Add the angle to the 180-degree list
            angles_180 = [angles_180, all_angles(turn_idx)];
        else
            % Add the angle to the 90-degree list
            angles_90 = [angles_90, all_angles(turn_idx)];
        end
    end

    % Compute the averages and standard deviations for 90-degree and 180-degree turns
    average_angles_90 = mean(angles_90);  % Average for all 90-degree turns
    std_angles_90 = std(angles_90, 1);       % Standard deviation for 90-degree turns
    average_angles_180 = mean(angles_180);  % Average for the 180-degree turn
    std_angles_180 = std(angles_180, 1);       % Standard deviation for the 180-degree turn
end

% Function to calculate the averages and standard deviations of the segments and angles for all files in a folder
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
    
    % Get a list of all .mat files in the folder
    files = dir(fullfile(folderPath, '*.mat'));
    
    % Initialize arrays to store all distances and angles from all files
    all_distances = [];
    all_angles_90 = [];
    all_angles_180 = [];
    
    % Loop through each .mat file and process it
    for file_idx = 1:length(files)
        % Load the current .mat file
        dataFile = fullfile(files(file_idx).folder, files(file_idx).name);
        load(dataFile);
        
        % Find the first loaded variable (assumes the data is the first variable)
        loadedVariables = who('-file', dataFile);
        loadedVariableName = loadedVariables{1}; 
        data = eval(loadedVariableName);
        
        % Extract trajectory data
        X = data.Trajectories.Labeled.Data(:, 1, :);
        Y = data.Trajectories.Labeled.Data(:, 2, :);
        X = squeeze(X)';  % Convert the X data into a 2D matrix
        Y = squeeze(Y)';  % Convert the Y data into a 2D matrix

        % Extract the labels directly from the dataset
        position_labels = data.Trajectories.Labeled.Labels;
        
        % Analyze the movement segments and angles using Movement_Analysis_Single_File
        [segment_distances, ~, ~, angles] = Movement_Analysis_Single_File(X, Y, min_distance, position_labels, label);
        
        % Filter out segments that are less than 800 mm
        valid_distances = segment_distances(segment_distances >= 800);
        
        % Split angles into 90-degree and 180-degree categories
        angles_90 = [];
        angles_180 = [];
        for turn_idx = 1:length(angles)
            if turn_idx == turn_number_for_180
                angles_180 = [angles_180, angles(turn_idx)];
            else
                angles_90 = [angles_90, angles(turn_idx)];
            end
        end
        
        % Collect valid distances and angles from this file
        all_distances = [all_distances, valid_distances];  % Only use valid distances
        all_angles_90 = [all_angles_90, angles_90];
        all_angles_180 = [all_angles_180, angles_180];
    end
    
    % Ensure there is valid data before calculating averages and standard deviations
    if isempty(all_distances)
        final_average_distance = NaN;
        final_std_distance = NaN;
        warning('No valid distances found (all below 800 mm).');
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

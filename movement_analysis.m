%% Analyse single file:

clear; 
clc;

dataFile = '../Project_tests/ekf_175wb/data/test9.mat';

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
label = 'tower';

[segment_distances, segment_labels, corners, angles] = Movement_Analysis_Single_File(X, Y, min_distance, position_labels, label);


% --- Plot raw path and detected corners ---
figure; hold on; grid on; axis equal;
xlabel('x position [mm]');
ylabel('y position [mm]');
title(sprintf('Raw OptiTrack path and detected corners for "%s"', label));

% Extract the data for the chosen label
pos_idx = find(strcmp(position_labels, label), 1);
plot(X(:, pos_idx), Y(:, pos_idx), 'b-', 'LineWidth', 1.2);

% Plot detected corners (RDP result)
if ~isempty(corners)
    scatter(corners(:,1), corners(:,2), 50, 'r', 'filled');
    for ci = 1:size(corners,1)
        text(corners(ci,1), corners(ci,2), sprintf('  %d', ci), ...
            'Color', 'r', 'FontWeight', 'bold');
    end
end

legend('Trajectory', 'Detected corners');


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

close all;

%% Analyse all files in a folder

clear;
clc;

% Folder containing all the .mat files
folderPath = '../Project_tests/ekf_175wb/data/';
    

% Define a minimum distance for segments to be considered valid (e.g., 800 mm)
min_distance = 700;
label = 'tower';  
% Specify the turn number that corresponds to the 180-degree turn (e.g., 4)
turn_number_for_180 = 4;
[final_average_distance, final_std_distance, final_average_90, final_std_90, ...
 final_average_180, final_std_180, per_turn_average_angles, per_turn_std_angles] = ...
    AnalyzeFolder(folderPath, min_distance, label, turn_number_for_180);

% Display the results
fprintf('Final Average Segment Distance: %.2f mm (std: %.2f mm)\n', final_average_distance, final_std_distance);
fprintf('Final Average of 90-degree turns: %.2f degrees (std: %.2f degrees)\n', final_average_90, final_std_90);
fprintf('Final Average of 180-degree turn: %.2f degrees (std: %.2f degrees)\n', final_average_180, final_std_180);
for turn_idx = 1:numel(per_turn_average_angles)
    fprintf('Average angle for turn %d: %.2f degrees (std: %.2f degrees)\n', ...
        turn_idx, per_turn_average_angles(turn_idx), per_turn_std_angles(turn_idx));
end


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
    [X,Y] = removeNaNValues(X,Y);
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
        % --- RDP-based corner detection (simple & robust) ---
        P = [x_data, y_data];

        % Tolerance (mm). Smaller => more corners, larger => fewer
        epsilon = 50;  % try 30–80 depending on noise / desired sensitivity
        idx_keep = rdp_indices(P, epsilon);

        % Corners are the kept points along the path
        corner_points = P(idx_keep, :);

        % Segment distances (gate by min_distance) and filter corners accordingly
        if size(corner_points,1) >= 2
            % distances between consecutive RDP corners
            seg_d = sqrt(sum(diff(corner_points,1,1).^2, 2));
            use = seg_d >= min_distance;     % keep segments above threshold

            % record distances/labels for kept segments
            if any(use)
                segment_distances = [segment_distances, seg_d(use)'];
                segment_labels = [segment_labels, repmat({label}, 1, nnz(use))];
            end

            % Build a filtered corner list that only contains corners belonging
            % to the kept segments (deduplicated, in order)
            kept_idx = find(use);
            corners_kept = [];
            for k = 1:numel(kept_idx)
                i = kept_idx(k);  % segment from corner i -> i+1
                if isempty(corners_kept)
                    corners_kept = [corners_kept; corner_points(i,:); corner_points(i+1,:)];
                else
                    % If current segment starts at the last kept corner, just append its end
                    if all(corners_kept(end,:) == corner_points(i,:))
                        corners_kept = [corners_kept; corner_points(i+1,:)];
                    else
                        % Gap in kept segments: start a new chain explicitly
                        corners_kept = [corners_kept; corner_points(i,:); corner_points(i+1,:)];
                    end
                end
            end

            % Replace with filtered corners for downstream usage (plot/angles)
            corner_points = corners_kept;
        end

        % Append only filtered corners and compute angles only from them
        if ~isempty(corner_points)
            corners = [corners; corner_points];
        end

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

function [final_average_distance, final_std_distance, final_average_90, final_std_90, ...
          final_average_180, final_std_180, per_turn_average_angles, per_turn_std_angles] = ...
          AnalyzeFolder(folderPath, min_distance, label, turn_number_for_180)
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
    %   per_turn_average_angles - Average angle for each turn index
    %   per_turn_std_angles - Standard deviation per turn index

    files = dir(fullfile(folderPath, '*.mat'));

    all_distances = [];
    all_angles_90 = [];
    all_angles_180 = [];
    angles_by_turn = {};

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
        for turn_idx = 1:length(angles)
            if numel(angles_by_turn) < turn_idx
                angles_by_turn{turn_idx} = []; %#ok<AGROW>
            end
            angles_by_turn{turn_idx}(end+1) = angles(turn_idx); %#ok<AGROW>
        end
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

    num_turns = numel(angles_by_turn);
    per_turn_average_angles = nan(1, num_turns);
    per_turn_std_angles = nan(1, num_turns);
    for turn_idx = 1:num_turns
        turn_angles = angles_by_turn{turn_idx};
        per_turn_average_angles(turn_idx) = mean(turn_angles);
        per_turn_std_angles(turn_idx) = std(turn_angles);
    end
end


function idx_keep = rdp_indices(P, eps)
% Ramer–Douglas–Peucker: return indices of kept points (corners)
% P   : robot trajectory [x y]
% eps : tolerance, make higher if false corners appear

    n = size(P,1);
    if n <= 2
        idx_keep = (1:n).';
        return;
    end

    idx_keep = [1; n];
    stack = [1, n];

    while ~isempty(stack)
        a = stack(end,1);
        b = stack(end,2);
        stack(end,:) = [];

        A = P(a,:); B = P(b,:);
        AB = B - A;
        AB2 = sum(AB.^2);

        maxd = 0; idx = -1;
        for i = a+1:b-1
            AP = P(i,:) - A;
            if AB2 == 0
                d = norm(AP);
            else
                t = max(0, min(1, dot(AP,AB)/AB2));
                proj = A + t*AB;
                d = norm(P(i,:) - proj);
            end
            if d > maxd
                maxd = d; idx = i;
            end
        end

        if maxd > eps
            idx_keep = [idx_keep; idx]; 
            stack = [stack; a, idx; idx, b]; 
        end
    end

    % Remove corners with small angles (angles < 20 degrees)
    min_angle_deg = 20;
    idx_keep = sort(idx_keep);

    while true
        corner_pts = P(idx_keep, :);
        ncp = size(corner_pts, 1);
        if ncp < 3
            break;
        end

        removed_any = false;
        for k = 2:ncp-1
            v1 = corner_pts(k, :) - corner_pts(k-1, :);
            v2 = corner_pts(k+1, :) - corner_pts(k, :);
            angle = atan2d(abs(det([v1; v2])), dot(v1, v2));
            if angle < min_angle_deg
                idx_keep(k) = []; 
                removed_any = true;
                break;            
            end
        end

        if ~removed_any
            break;
        end
    end

    idx_keep = sort(idx_keep);
end


function [X_clean, Y_clean] = removeNaNValues(X, Y)
    valid_rows = all(isfinite(X), 2) & all(isfinite(Y), 2);
    X_clean = X(valid_rows, :);
    Y_clean = Y(valid_rows, :);
end


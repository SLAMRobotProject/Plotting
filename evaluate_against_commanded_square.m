% Compare OptiTrack and odometry trajectories against a commanded 1x1 m square
% path. Loads matching MAT/CSV pairs (like evaluate_estimator_performance.m),
% aligns them, and reports geometric deviation metrics relative to the ideal
% square (both CW and CCW directions). Distances are computed geometrically,
% so no commanded timestamps are required.

clear; clc; close all;

%% SETTINGS
analyze_folder     = true; 
baseFolder         = '../Project_tests/ekf_175wb/data';
single_test_number = 5;        % used if analyze_folder == false
single_mat_file    = '';        % optional explicit override
single_csv_file    = '';
desired_label      = 'tower';   % OptiTrack marker label
square_size_m      = 1.0;       % commanded track dimension
plot_results       = true;

%% Locate files
if analyze_folder
    matFiles = dir(fullfile(baseFolder, '*.mat'));
    if isempty(matFiles)
        error('No MAT files found in folder: %s', baseFolder);
    end
    test_numbers = 1:numel(matFiles);
    filePairs = struct('mat', [], 'csv', []);
    for k = 1:numel(matFiles)
        matPath = fullfile(matFiles(k).folder, matFiles(k).name);
        [~, baseName, ~] = fileparts(matFiles(k).name);
        csvPath = fullfile(matFiles(k).folder, [baseName '.csv']);
        filePairs(k).mat = matPath;
        filePairs(k).csv = csvPath;
        if ~isfile(csvPath)
            warning('Missing CSV for %s (expected %s).', matFiles(k).name, csvPath);
        end
    end
    fprintf('Found %d MAT files in %s.\n', numel(matFiles), baseFolder);
else
    if ~isempty(single_mat_file) && ~isempty(single_csv_file)
        test_numbers = -1; % indicates explicit filenames used
    else
        test_numbers = single_test_number;
    end
end

nTests = numel(test_numbers);
opt_total_error      = nan(nTests,1);
odom_total_error     = nan(nTests,1);
opt_rmse_error       = nan(nTests,1);
odom_rmse_error      = nan(nTests,1);
opt_mean_error       = nan(nTests,1);
odom_mean_error      = nan(nTests,1);
opt_max_error        = nan(nTests,1);
odom_max_error       = nan(nTests,1);
ref_path = build_commanded_square_path(square_size_m);

for k = 1:nTests
    if analyze_folder
        dataFile = filePairs(k).mat;
        odometryFile = filePairs(k).csv;
        fprintf('\n========== Test %d: %s ==========\n', k, dataFile);
        if ~isfile(dataFile) || ~isfile(odometryFile)
            warning('Skipping test %d due to missing files.', k);
            continue;
        end
    else
        if ~isempty(single_mat_file) && ~isempty(single_csv_file)
            dataFile = single_mat_file;
            odometryFile = single_csv_file;
            fprintf('\n========== Explicit file set ==========\n');
        else
            dataFile = fullfile(baseFolder, sprintf('test%d.mat', test_numbers(k)));
            odometryFile = fullfile(baseFolder, sprintf('test%d.csv', test_numbers(k)));
            fprintf('\n========== Test %d ==========\n', test_numbers(k));
        end
    end

    % Load OptiTrack data (meters)
    disp(['Loading OptiTrack MAT: ', dataFile]);
    load(dataFile);
    matVars = who('-file', dataFile);
    data = eval(matVars{1}); %#ok<EVLDIR>
    labels = data.Trajectories.Labeled.Labels;
    if ~ismember(desired_label, labels)
        warning('Label %s not found in %s. Skipping.', desired_label, dataFile);
        continue;
    end
    X_all = squeeze(data.Trajectories.Labeled.Data(:, 1, :));
    Y_all = squeeze(data.Trajectories.Labeled.Data(:, 2, :));
    idxLabel = find(strcmp(labels, desired_label), 1);
    X_mm = X_all(:, idxLabel);
    Y_mm = Y_all(:, idxLabel);
    valid = isfinite(X_mm) & isfinite(Y_mm);
    X_mm = X_mm(valid);
    Y_mm = Y_mm(valid);
    x_opt = X_mm / 1000;
    y_opt = Y_mm / 1000;

    % Build OptiTrack timestamps
    ts_raw = data.Timestamp;
    if ischar(ts_raw) || isstring(ts_raw)
        parts = split(string(ts_raw));
        dt_start = datetime(erase(parts(1), ',' ) + " " + parts(2), ...
                            'InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
    else
        error('Unexpected Timestamp format in %s', dataFile);
    end
    if isfield(data, 'FrameRate')
        frameRate = data.FrameRate;
    elseif isfield(data.Trajectories.Labeled, 'FrameRate')
        frameRate = data.Trajectories.Labeled.FrameRate;
    else
        error('Frame rate not found in %s', dataFile);
    end
    N_opt = numel(x_opt);
    t_opt = seconds(timeofday(dt_start + seconds((0:N_opt-1)' ./ frameRate)));
    t_opt = t_opt(valid);

    % Load odometry data (meters)
    disp(['Loading odometry CSV: ', odometryFile]);
    odomTbl = readtable(odometryFile);
    t_odom = seconds(odomTbl{:,1});
    x_odom = table2array(odomTbl(:,3))*10/1000; % convert to meters
    y_odom = table2array(odomTbl(:,4))*10/1000;

    % Align by overlapping time window
    t0 = max(t_opt(1), t_odom(1));
    t1 = min(t_opt(end), t_odom(end));
    if t1 <= t0
        warning('No overlapping data window for %s. Skipping.', dataFile);
        continue;
    end
    mask_opt = (t_opt >= t0) & (t_opt <= t1);
    t_ref = t_opt(mask_opt);
    x_opt_use = x_opt(mask_opt);
    y_opt_use = y_opt(mask_opt);
    x_odom_interp = interp1(t_odom, x_odom, t_ref, 'linear');
    y_odom_interp = interp1(t_odom, y_odom, t_ref, 'linear');
    bad = isnan(x_odom_interp) | isnan(y_odom_interp);
    if any(bad)
        t_ref = t_ref(~bad);
        x_opt_use = x_opt_use(~bad);
        y_opt_use = y_opt_use(~bad);
        x_odom_interp = x_odom_interp(~bad);
        y_odom_interp = y_odom_interp(~bad);
    end

    % Align orientation so initial motion points along +y
    min_leg_dist = 0.15; % meters
    phi_opt = estimate_initial_direction(x_opt_use, y_opt_use, min_leg_dist);
    phi_odom = estimate_initial_direction(x_odom_interp, y_odom_interp, min_leg_dist);
    dphi = phi_opt - phi_odom;
    R_align = [cos(dphi), -sin(dphi); sin(dphi), cos(dphi)];
    p0_opt = [x_opt_use(1); y_opt_use(1)];
    p0_odom = [x_odom_interp(1); y_odom_interp(1)];
    odom_pts = [x_odom_interp.'; y_odom_interp.'];
    odom_rot = R_align * (odom_pts - p0_odom);
    odom_aligned = odom_rot + p0_opt;

    % Shift both to origin and rotate so heading along +y
    theta_plot = pi/2 - phi_opt;
    R_plot = [cos(theta_plot), -sin(theta_plot); sin(theta_plot), cos(theta_plot)];
    opt_rel = [x_opt_use.' - p0_opt(1); y_opt_use.' - p0_opt(2)];
    odom_rel = [odom_aligned(1,:) - p0_opt(1); odom_aligned(2,:) - p0_opt(2)];
    opt_plot = R_plot * opt_rel;
    odom_plot = R_plot * odom_rel;
    opt_xy = opt_plot.';
    odom_xy = odom_plot.';

    % Compare to commanded path (CW lap followed by CCW lap)
    opt_metrics = compare_to_square(opt_xy, ref_path);
    odom_metrics = compare_to_square(odom_xy, ref_path);

    opt_total_error(k) = opt_metrics.total_error;
    odom_total_error(k) = odom_metrics.total_error;
    opt_rmse_error(k) = opt_metrics.rmse_error;
    odom_rmse_error(k) = odom_metrics.rmse_error;
    opt_mean_error(k) = opt_metrics.mean_error;
    odom_mean_error(k) = odom_metrics.mean_error;
    opt_max_error(k) = opt_metrics.max_error;
    odom_max_error(k) = odom_metrics.max_error;
    fprintf('OptiTrack mean error: %.3f m, RMSE: %.3f m, max: %.3f m, total integral: %.3f m^2\n', ...
        opt_metrics.mean_error, opt_metrics.rmse_error, opt_metrics.max_error, ...
        opt_metrics.total_error);
    fprintf('Odometry  mean error: %.3f m, RMSE: %.3f m, max: %.3f m, total integral: %.3f m^2\n', ...
        odom_metrics.mean_error, odom_metrics.rmse_error, odom_metrics.max_error, ...
        odom_metrics.total_error);

    if plot_results
        figure('Name', sprintf('Square tracking error - %s', dataFile));
        hold on; axis equal; grid on;
        plot(ref_path(:,1), ref_path(:,2), 'k--', 'LineWidth', 1.2);
        plot(opt_xy(:,1), opt_xy(:,2), 'b-', 'DisplayName','OptiTrack');
        plot(odom_xy(:,1), odom_xy(:,2), 'm-', 'DisplayName','Odometry');
        plot(opt_xy(1,1), opt_xy(1,2), 'go', 'MarkerSize',8, 'DisplayName','Start');
        xlabel('x [m]'); ylabel('y [m]');
        title('Trajectories vs commanded square (start at origin, heading +y)');
        legend('Commanded square','OptiTrack','Odometry','Start','Location','best');
    end
end

%% Summary table
if nTests > 1
    testLabels = strings(nTests,1);
    for k = 1:nTests
        if analyze_folder
            [~, baseName, ext] = fileparts(filePairs(k).mat);
            testLabels(k) = string(baseName) + string(ext);
        else
            if ~isempty(single_mat_file)
                [~, baseName, ext] = fileparts(single_mat_file);
                testLabels(k) = string(baseName) + string(ext);
            else
                testLabels(k) = sprintf('test%d', test_numbers(k));
            end
        end
    end
    summaryTable = table(testLabels, opt_mean_error, opt_rmse_error, opt_max_error, opt_total_error, ...
                         odom_mean_error, odom_rmse_error, odom_max_error, odom_total_error, ...
        'VariableNames', {'Test','Opt_mean','Opt_RMSE','Opt_max','Opt_total_int', ...
                          'Odometry_mean','Odometry_RMSE','Odometry_max','Odometry_total_int'});
    disp(summaryTable);

    meanOptMean   = mean(opt_mean_error,  'omitnan');
    meanOptRMSE   = mean(opt_rmse_error,  'omitnan');
    meanOptMax    = mean(opt_max_error,   'omitnan');
    meanOptTotal  = mean(opt_total_error, 'omitnan');
    stdOptMean    = std(opt_mean_error,  'omitnan');
    stdOptRMSE    = std(opt_rmse_error,  'omitnan');
    stdOptMax     = std(opt_max_error,   'omitnan');
    stdOptTotal   = std(opt_total_error, 'omitnan');
    meanOdomMean  = mean(odom_mean_error,  'omitnan');
    meanOdomRMSE  = mean(odom_rmse_error,  'omitnan');
    meanOdomMax   = mean(odom_max_error,   'omitnan');
    meanOdomTotal = mean(odom_total_error, 'omitnan');
    stdOdomMean   = std(odom_mean_error,  'omitnan');
    stdOdomRMSE   = std(odom_rmse_error,  'omitnan');
    stdOdomMax    = std(odom_max_error,   'omitnan');
    stdOdomTotal  = std(odom_total_error, 'omitnan');

    fprintf('\n=== Aggregate metrics over %d tests ===\n', nTests);
    fprintf('OptiTrack: mean = %.3f m (std %.3f m), RMSE = %.3f m (std %.3f m), max = %.3f m (std %.3f m), total integral = %.3f m^2 (std %.3f m^2)\n', ...
        meanOptMean, stdOptMean, meanOptRMSE, stdOptRMSE, meanOptMax, stdOptMax, meanOptTotal, stdOptTotal);
    fprintf('Odometry : mean = %.3f m (std %.3f m), RMSE = %.3f m (std %.3f m), max = %.3f m (std %.3f m), total integral = %.3f m^2 (std %.3f m^2)\n', ...
        meanOdomMean, stdOdomMean, meanOdomRMSE, stdOdomRMSE, meanOdomMax, stdOdomMax, meanOdomTotal, stdOdomTotal);
end

%% Helper functions
function phi = estimate_initial_direction(x, y, min_dist)
    x = x(:); y = y(:);
    if numel(x) < 2
        phi = 0; return;
    end
    x0 = x(1); y0 = y(1);
    phi = 0;
    for k = 2:numel(x)
        dx = x(k) - x0;
        dy = y(k) - y0;
        if hypot(dx, dy) >= min_dist
            phi = atan2(dy, dx);
            return;
        end
    end
end


function ref_path = build_commanded_square_path(size_m)
    cw  = [0 0; size_m 0; size_m size_m; 0 size_m; 0 0];
    ccw = [0 0; 0 size_m; size_m size_m; size_m 0; 0 0];
    ref_path = [cw; ccw(2:end, :)];
end

function metrics = compare_to_square(traj_xy, ref_path)
    metrics = struct('mean_error',NaN,'rmse_error',NaN,'max_error',NaN,'total_error',NaN);
    valid = all(isfinite(traj_xy), 2);
    pts = traj_xy(valid, :);
    if size(pts,1) < 2
        return;
    end
    arc = [0; cumsum(vecnorm(diff(pts),2,2))];
    [distances, ~] = distance_to_polyline(pts, ref_path);
    metrics.total_error = trapz(arc, distances);
    metrics.mean_error = mean(distances);
    metrics.rmse_error = sqrt(mean(distances.^2));
    metrics.max_error = max(distances);
end

function [distances, segment_idx] = distance_to_polyline(points, polyline)
    n_pts = size(points,1);
    n_seg = size(polyline,1) - 1;
    distances = zeros(n_pts,1);
    segment_idx = zeros(n_pts,1);
    for p = 1:n_pts
        best_d = inf; best_seg = 1;
        P = points(p, :);
        for s = 1:n_seg
            A = polyline(s, :);
            B = polyline(s+1, :);
            AB = B - A;
            denom = dot(AB, AB);
            if denom < eps
                d = norm(P - A);
                proj = A;
            else
                t = max(0, min(1, dot(P - A, AB) / denom));
                proj = A + t * AB;
                d = norm(P - proj);
            end
            if d < best_d
                best_d = d;
                best_seg = s;
            end
        end
        distances(p) = best_d;
        segment_idx(p) = best_seg;
    end
end

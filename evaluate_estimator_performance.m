% File for evaluating estimator performance against OptiTrack ground truth
% data. Compares position estimates from odometry (CSV) against
% OptiTrack (MAT) data for multiple test runs.
% Can run over all test*.mat files in a folder or a single specified test.
% Calculates RMSE, mean error, final error, and plots trajectories and errors.

clear; clc; close all;

%% SETTINGS
analyze_folder     = true; 

baseFolder         = '../Project_tests/ekf_175wb/data/';
single_test_number = 5;      % used if analyze_folder == false, reads file as testN.mat / testN.csv
desired_label      = 'tower';   % OptiTrack marker label

% Can set explicit filenames here
single_mat_file  = '../Project_tests/increased_wb/data/test8.mat';
single_csv_file  = '../Project_tests/increased_wb/data/test8.csv';

if analyze_folder
    % Automatically find all .mat files in the folder
    matFiles = dir(fullfile(baseFolder, '*.mat'));

    if isempty(matFiles)
        error('No MAT files found in folder: %s', baseFolder);
    end 

    % List of files
    test_numbers = 1:numel(matFiles);
    filePairs = struct([]);

    for k = 1:numel(matFiles)
        matPath = fullfile(matFiles(k).folder, matFiles(k).name);

        [~, baseName, ~] = fileparts(matFiles(k).name);
        csvCandidate = fullfile(matFiles(k).folder, [baseName '.csv']);

        if isfile(csvCandidate)
            filePairs(k).mat = matPath;
            filePairs(k).csv = csvCandidate;
        else
            warning('No matching CSV file for MAT file: %s', matFiles(k).name);
            filePairs(k).mat = matPath;
            filePairs(k).csv = '';
        end
    end
    fprintf('Found %d MAT file(s) in folder %s.\n', numel(matFiles), baseFolder);
else
    fprintf('Running single test mode.\n');

    if ~isempty(single_mat_file) && ~isempty(single_csv_file)
        % Uses explicit filenames here
        test_numbers = -1;  % to indicate explicit files
        fprintf('Using explicit files:\n MAT = %s\n CSV = %s\n', single_mat_file, single_csv_file);
    else
        % Use numbered files
        test_numbers = single_test_number;
        fprintf('Using numbered files: test%d.mat / test%d.csv\n', single_test_number, single_test_number);
    end
end

nTests = numel(test_numbers);

rmse_pos_all      = nan(nTests,1);
mean_pos_all      = nan(nTests,1);
std_pos_all       = nan(nTests,1);
p95_pos_all       = nan(nTests,1);
final_err_all     = nan(nTests,1);
path_len_all      = nan(nTests,1);
rmse_norm_all     = nan(nTests,1);
samples_all       = nan(nTests,1);

for k = 1:nTests
    test_number = test_numbers(k);
    fprintf('\n========== Running test %d ==========\n', test_number);
    if analyze_folder
        dataFile     = filePairs(k).mat;
        odometryFile = filePairs(k).csv;

        if isempty(odometryFile)
            error('No CSV file found for %s', dataFile);
        end
    else
        if ~isempty(single_mat_file) && ~isempty(single_csv_file)
            dataFile     = single_mat_file;
            odometryFile = single_csv_file;
        else
            % Fall back to numeric convention: testN.mat and testN.csv
            dataFile     = fullfile(baseFolder, sprintf('test%d.mat',  test_number));
            odometryFile = fullfile(baseFolder, sprintf('test%d.csv', test_number));
        end
    end


%% Load optitrack data
disp(['Loading OptiTrack MAT: ', dataFile]);
load(dataFile);

loadedVariables    = who('-file', dataFile);
loadedVariableName = loadedVariables{1};
data               = eval(loadedVariableName);  %#ok<EVLDIR>

position_labels = data.Trajectories.Labeled.Labels;

if ~ismember(desired_label, position_labels)
    error('Label "%s" not found. Available: %s', ...
        desired_label, strjoin(position_labels, ', '));
end

% reading positions from OptiTrack data
X_all = squeeze(data.Trajectories.Labeled.Data(:, 1, :)); 
Y_all = squeeze(data.Trajectories.Labeled.Data(:, 2, :));
idxMarker = find(strcmp(position_labels, desired_label), 1);
X_mm = X_all(:, idxMarker);
Y_mm = Y_all(:, idxMarker);

% remove nan values
good = isfinite(X_mm) & isfinite(Y_mm);
X_mm = X_mm(good);
Y_mm = Y_mm(good);

% Convert to meters
x_opt = X_mm / 1000.0; 
y_opt = Y_mm / 1000.0; 

% Get timestamps for optitrack, start time + frame rate 
ts_raw = data.Timestamp;  

if ischar(ts_raw) || isstring(ts_raw)
    parts    = split(string(ts_raw));
    date_str = erase(parts(1), ',');
    time_str = parts(2);          
    dt_start = datetime(date_str + " " + time_str, ...
                        'InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
else
    error('Unexpected type for data.Timestamp');
end

% Frame rate
if isfield(data, 'FrameRate')
    frameRate = data.FrameRate;
elseif isfield(data.Trajectories.Labeled, 'FrameRate')
    frameRate = data.Trajectories.Labeled.FrameRate;
else
    error('Could not find frame rate in OptiTrack data.');
end

N_opt    = numel(x_opt);
t_opt_dt = dt_start + seconds((0:N_opt-1)' ./ frameRate);
t_opt    = seconds(timeofday(t_opt_dt));   % seconds-of-day
t_opt = t_opt(good);

%% Load odometry data 
disp(['Loading odometry CSV: ', odometryFile]);
odomTbl = readtable(odometryFile);

t_odom_raw  = odomTbl{:,1};                  % duration or similar
x_odom_mm   = table2array(odomTbl(:,3))*10;  % your *10 -> mm
y_odom_mm   = table2array(odomTbl(:,4))*10;

% Convert odometry pos to meters
x_odom = x_odom_mm / 1000.0;
y_odom = y_odom_mm / 1000.0; 

t_odom = seconds(t_odom_raw);  

%% Only look at overlapping time interval between OptiTrack and odometry
t0 = max(t_opt(1),  t_odom(1));
t1 = min(t_opt(end), t_odom(end));

if t1 <= t0
    error('No overlapping time interval between OptiTrack and odometry.');
end

% Use OptiTrack as reference timeline on [t0, t1]
mask_opt   = (t_opt >= t0) & (t_opt <= t1);
t_ref      = t_opt(mask_opt);
x_opt_use  = x_opt(mask_opt);
y_opt_use  = y_opt(mask_opt);

% Interpolate odometry onto OptiTrack timeline, linear interpolation
x_odom_interp = interp1(t_odom, x_odom, t_ref, 'linear');
y_odom_interp = interp1(t_odom, y_odom, t_ref, 'linear');

bad = isnan(x_odom_interp) | isnan(y_odom_interp);
if any(bad)
    t_ref          = t_ref(~bad);
    x_opt_use      = x_opt_use(~bad);
    y_opt_use      = y_opt_use(~bad);
    x_odom_interp  = x_odom_interp(~bad);
    y_odom_interp  = y_odom_interp(~bad);
end


%% Rotate so plots align
min_leg_dist = 0.15;  

phi_opt  = estimate_initial_direction(x_opt_use,     y_opt_use,     min_leg_dist);
phi_odom = estimate_initial_direction(x_odom_interp, y_odom_interp, min_leg_dist);

dphi = phi_opt - phi_odom;  % rotation to apply to odometry

R = [cos(dphi), -sin(dphi);
     sin(dphi),  cos(dphi)];

% Start points at the first sample in the overlapping window
p0_opt  = [x_opt_use(1);     y_opt_use(1)];
p0_odom = [x_odom_interp(1); y_odom_interp(1)];

% Shift odometry to its own origin, rotate, then shift to OptiTrack origin
odom_pts     = [x_odom_interp.'; y_odom_interp.'];
odom_rel     = odom_pts - p0_odom;
odom_rot     = R * odom_rel;
odom_aligned = odom_rot + p0_opt;

x_odom_aligned = odom_aligned(1, :).';
y_odom_aligned = odom_aligned(2, :).';

% Start points after alignment
p_opt_start  = p0_opt;
p_odom_start = [x_odom_aligned(1); y_odom_aligned(1)];

%% Calculate position errors and metrics

dx        = x_odom_aligned - x_opt_use;
dy        = y_odom_aligned - y_opt_use;
pos_error = sqrt(dx.^2 + dy.^2); 

rmse_pos       = sqrt(mean(pos_error.^2));
mean_pos       = mean(pos_error);
std_pos        = std(pos_error);
p95_pos        = prctile(pos_error, 95);
dx_final       = x_odom_aligned(end) - x_opt_use(end);
dy_final       = y_odom_aligned(end) - y_opt_use(end);
final_pos_err  = sqrt(dx_final^2 + dy_final^2);

% Path length from optitrack data
dx_path     = diff(x_opt_use);
dy_path     = diff(y_opt_use);
seg_lengths = sqrt(dx_path.^2 + dy_path.^2);
path_length = sum(seg_lengths);
rmse_pos_norm = rmse_pos / max(path_length, 1e-6);

samples_all(k)   = numel(t_ref);
rmse_pos_all(k)  = rmse_pos;
mean_pos_all(k)  = mean_pos;
std_pos_all(k)   = std_pos;
p95_pos_all(k)   = p95_pos;
final_err_all(k) = final_pos_err;
path_len_all(k)  = path_length;
rmse_norm_all(k) = rmse_pos_norm;

%% Print results

fprintf('\n=== Estimator vs OptiTrack (initial pose aligned, interpolated) ===\n');
fprintf('Samples used:            %d\n', numel(t_ref));
fprintf('Path length (OptiTrack): %.3f m\n\n', path_length);

fprintf('Position RMSE:           %.3f m (%.2f %% of path length)\n', ...
    rmse_pos, rmse_pos_norm*100);
fprintf('Position mean error:     %.3f m (std: %.3f m)\n', ...
    mean_pos, std_pos);
fprintf('Position 95th perc.:     %.3f m\n', p95_pos);
fprintf('Final position error:    %.3f m\n', final_pos_err);


%% Align data with start point at origin and initial heading along +y for plotting
 
% Use OptiTrack start as common origin
p0_plot = p_opt_start;  

% Shift both trajectories to this origin
opt_rel  = [x_opt_use.'      - p0_plot(1);
            y_opt_use.'      - p0_plot(2)];
odom_rel = [x_odom_aligned.' - p0_plot(1);
            y_odom_aligned.' - p0_plot(2)];

theta_plot = pi/2 - phi_opt;
R_plot = [cos(theta_plot), -sin(theta_plot);
          sin(theta_plot),  cos(theta_plot)];

opt_plot  = R_plot * opt_rel;
odom_plot = R_plot * odom_rel;

x_opt_plot  = opt_plot(1,:).';
y_opt_plot  = opt_plot(2,:).';
x_odom_plot = odom_plot(1,:).';
y_odom_plot = odom_plot(2,:).';

p_opt_plot  = [x_opt_plot(1);  y_opt_plot(1)];
p_odom_plot = [x_odom_plot(1); y_odom_plot(1)];

%% Plot results

figure('Name', sprintf('Trajectories & position error - %s', dataFile));

subplot(1,2,1); hold on; axis equal;

h_opt   = plot(x_opt_plot,  y_opt_plot,  'b-',  'DisplayName','OptiTrack');
h_odom  = plot(x_odom_plot, y_odom_plot, 'm--', 'DisplayName','Odometry (aligned)');

h_opt_start  = plot(p_opt_plot(1),  p_opt_plot(2),  'go', 'MarkerSize',8, 'LineWidth',1, ...
                    'DisplayName','OptiTrack start');
h_odom_start = plot(p_odom_plot(1), p_odom_plot(2), 'ro', 'MarkerSize',8, 'LineWidth',1, ...
                    'DisplayName','Odom start (aligned)');

xlabel('x_{plot} [m]'); ylabel('y_{plot} [m]');
title('Trajectory (start at origin, heading = +y)');
legend([h_opt_start, h_odom_start, h_opt, h_odom], ...
       {'OptiTrack start', 'Odom start', 'OptiTrack path', 'Odom path'}, ...
       'Location','best');
grid on;

subplot(1,2,2);
plot(t_ref - t_ref(1), pos_error*1000);  
xlabel('t [s]'); ylabel('Position error [mm]');
title('Position error vs time');
grid on;

end 

%% Print summary over all tests
if nTests > 1
    fprintf('\n========== SUMMARY OVER %d TESTS ==========\n', nTests);

    fileNames = strings(nTests,1);
    for i = 1:nTests
        if analyze_folder
            [~, bn, ext] = fileparts(filePairs(i).mat);
        else
            if ~isempty(single_mat_file)
                [~, bn, ext] = fileparts(single_mat_file);
            else
                matPath = fullfile(baseFolder, sprintf('test%d.mat', test_numbers(i)));
                [~, bn, ext] = fileparts(matPath);
            end
        end
        fileNames(i) = string(bn) + string(ext);
    end

    % Convert relative RMSE to percentage
    rmse_rel_percent = rmse_norm_all * 100;

    resultsTable = table(fileNames, samples_all, path_len_all, ...
                         rmse_pos_all, rmse_norm_all, rmse_rel_percent, ...
                         final_err_all, ...
        'VariableNames', {'File','NSamples','PathLength_m', ...
                          'RMSE_m','RMSE_rel','RMSE_rel_percent','FinalErr_m'});
    disp(resultsTable);

    % Mean and std for key metrics
    mean_rmse     = mean(rmse_pos_all,  'omitnan');
    std_rmse      = std(rmse_pos_all,   0, 'omitnan');
    mean_final    = mean(final_err_all, 'omitnan');
    std_final     = std(final_err_all,  0, 'omitnan');
    mean_rmse_rel = mean(rmse_norm_all, 'omitnan');
    std_rmse_rel  = std(rmse_norm_all,  0, 'omitnan');

    fprintf('\nRMSE position:     mean = %.3f m, std = %.3f m\n', mean_rmse, std_rmse);
    fprintf('Final pos error:   mean = %.3f m, std = %.3f m\n', mean_final, std_final);
    fprintf('RMSE (rel/path):   mean = %.3f %% , std = %.3f %%\n', ...
            mean_rmse_rel*100, std_rmse_rel*100);
end


function phi = estimate_initial_direction(x, y, min_dist)
% Estimate direction of the initial movement segment
    x = x(:); y = y(:);
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

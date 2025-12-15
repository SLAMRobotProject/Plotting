% Plots QTM ground truth vs estimated position trajectories.
% For 'square' mode: splits into CW/CCW segments and aligns to commanded path.
% For 'general' mode: plots full trajectory only.

clear; close all; clc;

%% Settings
mode = 'batch'; % 'single' or 'batch'
trajectory_type = "square"; % 'square' or 'general'
test_number = 1;
label = "tower";
square_size_mm = 1000;
manual_split_percent = 0; % set to 0 for automatic detection of when to split CW/CCW, else set percentage (0-100)

datafolder = '../Project_tests/ekf_175wb/data/';
outputfolder = '../Project_tests/ekf_175wb/new_plots/';
if ~exist(outputfolder, 'dir'); mkdir(outputfolder); end

%% Main
if strcmp(mode, 'batch')
    matfiles = dir(fullfile(datafolder, '*.mat'));
    fprintf('Found %d .mat files in %s\n', length(matfiles), datafolder);
    for i = 1:length(matfiles)
        [~, basename, ~] = fileparts(matfiles(i).name);
        datafile = fullfile(datafolder, matfiles(i).name);
        eststatefile = fullfile(datafolder, [basename, '.csv']);
        if ~exist(eststatefile, 'file')
            fprintf('WARNING: No matching CSV for %s, skipping...\n', matfiles(i).name);
            continue;
        end
        fprintf('\n=== Processing %s ===\n', basename);
        try
            process_test(datafile, eststatefile, outputfolder, label, square_size_mm, manual_split_percent, trajectory_type);
        catch ME
            fprintf('ERROR processing %s: %s\n', basename, ME.message);
        end
    end
    fprintf('\n=== Batch processing complete ===\n');
else
    datafile = fullfile(datafolder, sprintf('test%d.mat', test_number));
    eststatefile = fullfile(datafolder, sprintf('test%d.csv', test_number));
    process_test(datafile, eststatefile, outputfolder, label, square_size_mm, manual_split_percent, trajectory_type);
end

function process_test(datafile, eststatefile, outputfolder, label, square_size_mm, manual_split, traj_type)
    [traj_x, traj_y, est_x, est_y, basename] = load_data(datafile, eststatefile, label, traj_type);
    plot_full(traj_x, traj_y, est_x, est_y, basename, label, outputfolder, traj_type, square_size_mm);
    if traj_type == "square"
        plot_cw_ccw_subplots(traj_x, traj_y, est_x, est_y, basename, label, square_size_mm, manual_split, outputfolder);
    end
end

function [traj_x, traj_y, est_x, est_y, basename] = load_data(datafile, eststatefile, label, traj_type)
    disp("Loading: " + datafile);
    S = load(datafile);
    vars = fieldnames(S);
    data = S.(vars{1});
    
    labels = string(data.Trajectories.Labeled.Labels);
    if ~any(labels == string(label))
        error('Label "%s" not found. Available: %s', label, strjoin(labels, ", "));
    end
    
    X = squeeze(data.Trajectories.Labeled.Data(:,1,:))';
    Y = squeeze(data.Trajectories.Labeled.Data(:,2,:))';
    if numel(labels) > 1
        idx = find(labels == string(label), 1, 'first');
        traj_x_raw = X(:,idx); traj_y_raw = Y(:,idx);
    else
        traj_x_raw = X(:); traj_y_raw = Y(:);
    end
    
    disp("Loading: " + eststatefile);
    estT = readtable(eststatefile);
    est_x = table2array(estT(:,3)) * 10;
    est_y = table2array(estT(:,4)) * 10;
    
    if traj_type == "square"
        [traj_x0, traj_y0] = recenter(traj_x_raw, traj_y_raw);
        est_xy = ([0 1; -1 0] * [est_x(:)'; est_y(:)'])';
        [est_x0, est_y0] = recenter(est_xy(:,1), est_xy(:,2));
        
        phi_qtm = get_heading(traj_x0, traj_y0, 150);
        phi_est = get_heading(est_x0, est_y0, 150);
        R0 = rotmat(phi_qtm - phi_est);
        est_aligned = (R0 * [est_x0(:)'; est_y0(:)'])';
        
        angle = get_initial_rotation(est_aligned(:,1), est_aligned(:,2)) + pi/2;
        Rvis = rotmat(angle);
        traj_vis = (Rvis * [traj_x0(:)'; traj_y0(:)'])';
        est_vis = (Rvis * est_aligned')';
        
        traj_x = traj_vis(:,1); traj_y = traj_vis(:,2);
        est_x = est_vis(:,1); est_y = est_vis(:,2);
    else
        [traj_x, traj_y] = recenter(traj_x_raw, traj_y_raw);
        [est_x, est_y] = recenter(est_x, est_y);
    end
    [~, basename, ~] = fileparts(datafile);
end

function plot_full(traj_x, traj_y, est_x, est_y, basename, label, outputfolder, traj_type, L)
    fig = figure('Name', sprintf('%s: Full trajectory', basename));
    hold on; grid on; axis equal;
    
    if strcmp(traj_type, 'square')
        cmd = get_square_paths(L);
        plot(cmd{1}(:,1), cmd{1}(:,2), 'k:', 'LineWidth', 1.5, 'DisplayName', 'Commanded path');
        plot(cmd{2}(:,1), cmd{2}(:,2), 'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    
    plot(0, 0, 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
    plot(traj_x, traj_y, 'b-', 'LineWidth', 1.2, 'DisplayName', 'QTM (ground truth)');
    plot(est_x, est_y, 'm-', 'LineWidth', 1.2, 'DisplayName', 'Estimated postition');
    
    title('2D robot trajectory'); xlabel('x position [mm]'); ylabel('y position [mm]');
    legend('Location', 'best');
    xlim(get_limits([traj_x; est_x])); ylim(get_limits([traj_y; est_y]));
    
    savefile = fullfile(outputfolder, sprintf('%s_%s_full_trajectory.png', basename, label));
    disp("Saving: " + savefile); saveas(fig, savefile);
end

function plot_cw_ccw_subplots(traj_x, traj_y, est_x, est_y, basename, label, L, manual_split, outputfolder)
    seg_traj = split_loops(traj_x, traj_y, manual_split);
    seg_est = split_loops(est_x, est_y, manual_split);
    if isempty(seg_traj) || isempty(seg_est)
        error("Could not split CW/CCW segments.");
    end
    
    cmd = get_square_paths(L);
    fig = figure('Name', sprintf('%s: CW + CCW squares', basename));
    ax1 = subplot(1,2,1); hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
    ax2 = subplot(1,2,2); hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
    
    out1 = plot_segment(ax1, cmd{1}, traj_x, traj_y, seg_traj{1}, est_x, est_y, seg_est{1});
    out2 = plot_segment(ax2, cmd{2}, traj_x, traj_y, seg_traj{2}, est_x, est_y, seg_est{2});
    
    lims_x = get_limits([out1.allX; out2.allX]);
    lims_y = get_limits([out1.allY; out2.allY]);
    xlim(ax1, lims_x); ylim(ax1, lims_y);
    xlim(ax2, lims_x); ylim(ax2, lims_y);
    
    title(ax1, sprintf('CW square (%s)', label));
    title(ax2, sprintf('CCW square aligned to command (%s)', label));
    xlabel(ax1,'x position [mm]'); ylabel(ax1,'y position [mm]');
    xlabel(ax2,'x position [mm]'); ylabel(ax2,'y position [mm]');
    legend(ax1, out1.handles, {'segment start','commanded square','QTM (ground truth)','Estimated position'}, 'Location','best');
    
    savefile = fullfile(outputfolder, sprintf('%s_%s_CW_CCW_subplots.png', basename, label));
    disp("Saving: " + savefile); saveas(fig, savefile);
    
    fprintf('\n=== Corner Analysis Results ===\n');
    print_corners('CW - QTM', out1.qtm_corners, out1.qtm_angles, out1.qtm_dist);
    print_corners('CW - Est', out1.est_corners, out1.est_angles, out1.est_dist);
    print_corners('CCW - QTM', out2.qtm_corners, out2.qtm_angles, out2.qtm_dist);
    print_corners('CCW - Est', out2.est_corners, out2.est_angles, out2.est_dist);
end

function print_corners(name, corners, angles, distances)
    fprintf('%s:\n', name);
    if isempty(corners); fprintf('  No corners detected\n'); return; end
    fprintf('  Detected %d corners\n', size(corners, 1));
    for i = 1:length(distances); fprintf('  Segment %d: %.2f mm\n', i, distances(i)); end
    for i = 1:length(angles); fprintf('  Angle %d: %.2f deg\n', i+1, angles(i)); end
end

function out = plot_segment(ax, cmd_path, traj_x, traj_y, idx_traj, est_x, est_y, idx_est)
    qtm = [traj_x(idx_traj), traj_y(idx_traj)] - [traj_x(idx_traj(1)), traj_y(idx_traj(1))];
    est = [est_x(idx_est), est_y(idx_est)] - [est_x(idx_est(1)), est_y(idx_est(1))];
    
    phi_seg = get_heading(est(:,1), est(:,2), 100);
    phi_cmd = atan2(cmd_path(2,2)-cmd_path(1,2), cmd_path(2,1)-cmd_path(1,1));
    R = rotmat(phi_cmd - phi_seg);
    
    qtm_r = (R * qtm')'; qtm_r = qtm_r - qtm_r(1,:);
    est_r = (R * est')'; est_r = est_r - est_r(1,:);
    
    [out.qtm_corners, out.qtm_dist, out.qtm_angles] = detect_corners(qtm_r, 50, 700, 70);
    [out.est_corners, out.est_dist, out.est_angles] = detect_corners(est_r, 50, 700, 70);
    
    out.handles = [
        plot(ax, 0, 0, 'go', 'MarkerSize', 7, 'LineWidth', 1)
        plot(ax, cmd_path(:,1), cmd_path(:,2), 'k:', 'LineWidth', 1.2)
        plot(ax, qtm_r(:,1), qtm_r(:,2), 'b-', 'LineWidth', 1.2)
        plot(ax, est_r(:,1), est_r(:,2), 'm-', 'LineWidth', 1.2)
    ];
    out.allX = [cmd_path(:,1); qtm_r(:,1); est_r(:,1)];
    out.allY = [cmd_path(:,2); qtm_r(:,2); est_r(:,2)];
end

function [corners, distances, angles] = detect_corners(P, epsilon, min_dist, min_angle)
    corners = []; distances = []; angles = [];
    P = P(all(isfinite(P), 2), :);
    if size(P, 1) < 3; return; end
    
    idx = rdp(P, epsilon, min_angle);
    pts = P(idx, :);
    
    if size(pts, 1) >= 2
        seg_d = sqrt(sum(diff(pts).^2, 2));
        use = seg_d >= min_dist;
        if any(use); distances = seg_d(use)'; end
        
        kept = find(use);
        corners = [];
        for k = 1:numel(kept)
            i = kept(k);
            if isempty(corners)
                corners = [pts(i,:); pts(i+1,:)];
            elseif all(corners(end,:) == pts(i,:))
                corners = [corners; pts(i+1,:)];
            else
                corners = [corners; pts(i,:); pts(i+1,:)];
            end
        end
        pts = corners;
    end
    corners = pts;
    
    if size(pts, 1) > 2
        for i = 2:size(pts,1)-1
            v1 = pts(i,:) - pts(i-1,:);
            v2 = pts(i+1,:) - pts(i,:);
            angles = [angles, atan2d(norm(det([v1;v2])), dot(v1,v2))];
        end
    end
end

function idx = rdp(P, eps, min_angle)
    n = size(P, 1);
    if n <= 2; idx = (1:n)'; return; end
    
    idx = [1; n];
    stack = [1, n];
    while ~isempty(stack)
        a = stack(end,1); b = stack(end,2);
        stack(end,:) = [];
        A = P(a,:); B = P(b,:);
        AB = B - A; AB2 = sum(AB.^2);
        
        maxd = 0; best = -1;
        for i = a+1:b-1
            AP = P(i,:) - A;
            if AB2 == 0; d = norm(AP);
            else
                t = max(0, min(1, dot(AP,AB)/AB2));
                d = norm(P(i,:) - (A + t*AB));
            end
            if d > maxd; maxd = d; best = i; end
        end
        if maxd > eps
            idx = [idx; best];
            stack = [stack; a, best; best, b];
        end
    end
    idx = sort(idx);
    
    while true
        pts = P(idx,:);
        if size(pts,1) < 3; break; end
        removed = false;
        for k = 2:size(pts,1)-1
            v1 = pts(k,:) - pts(k-1,:);
            v2 = pts(k+1,:) - pts(k,:);
            if atan2d(abs(det([v1;v2])), dot(v1,v2)) < min_angle
                idx(k) = []; removed = true; break;
            end
        end
        if ~removed; break; end
    end
    idx = sort(idx);
end

function [x, y] = recenter(x, y)
    x = x(:); y = y(:);
    good = isfinite(x) & isfinite(y);
    if any(good)
        k = find(good, 1);
        x = x - x(k); y = y - y(k);
    end
end

function phi = get_heading(x, y, min_dist)
    x = x(:); y = y(:); phi = 0;
    if numel(x) < 2; return; end
    for k = 2:numel(x)
        if hypot(x(k)-x(1), y(k)-y(1)) >= min_dist
            phi = atan2(y(k)-y(1), x(k)-x(1)); return;
        end
    end
end

function angle = get_initial_rotation(x, y)
    x = x(:); y = y(:);
    good = isfinite(x) & isfinite(y);
    x = x(good); y = y(good);
    if numel(x) < 3; angle = 0; return; end
    x = x - x(1); y = y - y(1);
    
    idx = rdp([x, y], 50, 20);
    if numel(idx) < 2; angle = 0; return; end
    dx = x(idx(2)) - x(idx(1));
    dy = y(idx(2)) - y(idx(1));
    if hypot(dx, dy) < 1e-6; angle = 0; return; end
    angle = -atan2(dy, dx);
end

function segs = split_loops(x, y, manual_pct)
    if nargin < 3; manual_pct = 0; end
    x = x(:); y = y(:);
    segs = {};
    
    good = isfinite(x) & isfinite(y);
    x = x(good); y = y(good);
    n = numel(x);
    if n < 20; return; end
    
    if manual_pct > 0
        split = max(10, min(n-10, round(n * manual_pct / 100)));
        segs = {1:split, split:n};
        fprintf('Manual split at %.1f%% (index %d / %d)\n', manual_pct, split, n);
        return;
    end
    
    arc = [0; cumsum(hypot(diff(x), diff(y)))];
    total = arc(end);
    if total <= 1e-6; return; end
    
    dist_start = hypot(x - x(1), y - y(1));
    candidates = find(dist_start < 200 & arc > total*0.35 & arc < total*0.65);
    
    if ~isempty(candidates)
        [~, best] = min(dist_start(candidates));
        split = candidates(best);
    else
        mid_range = round(n*0.35):round(n*0.65);
        if ~isempty(mid_range)
            [~, best] = min(dist_start(mid_range));
            split = mid_range(best);
        else
            split = round(n/2);
        end
    end
    
    split = max(10, min(n-10, split));
    segs = {1:split, split:n};
    fprintf('Auto split at index %d / %d (%.1f%%)\n', split, n, 100*split/n);
end

function paths = get_square_paths(L)
    paths = {[0 0; 0 L; L L; L 0; 0 0], [0 0; L 0; L L; 0 L; 0 0]};
end

function lim = get_limits(v, pad)
    if nargin < 2; pad = 150; end
    v = v(isfinite(v));
    if isempty(v); lim = [-500, 500]; return; end
    lim = [min(v)-pad, max(v)+pad];
    if lim(1) == lim(2); lim = lim + [-1 1]*(max(abs(lim(1)),1)+pad); end
end

function R = rotmat(a)
    R = [cos(a), -sin(a); sin(a), cos(a)];
end
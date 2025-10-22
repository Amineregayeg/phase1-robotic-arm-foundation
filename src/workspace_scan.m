function [results, metrics] = workspace_scan(C, visualize)
% WORKSPACE_SCAN - Compute reachable workspace using joint space sampling
%
% Performs uniform sampling of joint space to determine the reachable
% end-effector workspace. Computes volume, coverage, and generates
% visualization plots.
%
% Inputs:
%   C         - Configuration struct from config()
%   visualize - Boolean, if true generates plots (default: true)
%
% Outputs:
%   results - Struct containing:
%       .points       - Reachable end-effector positions (N×3)
%       .volume       - Convex hull volume (m³)
%       .tray_coverage - Fraction of tray grid covered [0-1]
%       .grid_reachable - Boolean grid of reachable cells (gridNy × gridNx)
%       .samples      - Number of samples used
%   metrics - Struct with detailed metrics for validation
%
% Method: Sobol quasi-random sampling for uniform coverage
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    % Input validation
    if nargin < 1
        C = config();
    end
    if nargin < 2
        visualize = true;
    end

    % Set random seed for reproducibility
    rng(C.seed);

    % Generate samples using Sobol sequence for better coverage
    % (fallback to uniform random if Sobol not available)
    n_samples = C.workspace_samples;
    n_joints = C.n;

    try
        % Try Sobol sequence (better coverage)
        sobol_seq = sobolset(n_joints, 'Skip', 1000);
        samples_unit = net(sobol_seq, n_samples);  % [0,1]^n
    catch
        % Fallback to Latin hypercube or uniform random
        warning('workspace_scan:SobolUnavailable', 'Sobol sequence unavailable, using uniform random sampling');
        samples_unit = rand(n_samples, n_joints);
    end

    % Scale to joint limits
    q_samples = zeros(n_samples, n_joints);
    for i = 1:n_joints
        q_samples(:, i) = C.qmin(i) + samples_unit(:, i) * (C.qmax(i) - C.qmin(i));
    end

    % Compute forward kinematics for all samples
    fprintf('Computing workspace from %d samples...\n', n_samples);
    points = zeros(n_samples, 3);
    valid_count = 0;

    for i = 1:n_samples
        [T, ~, valid] = fk(q_samples(i, :)', C);
        if valid
            points(i, :) = T(1:3, 4)';
            valid_count = valid_count + 1;
        else
            points(i, :) = [NaN, NaN, NaN];
        end

        if mod(i, 10000) == 0
            fprintf('  Progress: %d/%d samples (%.1f%%)\n', i, n_samples, 100*i/n_samples);
        end
    end

    % Remove invalid points
    valid_idx = ~isnan(points(:, 1));
    points = points(valid_idx, :);

    fprintf('Valid samples: %d/%d (%.1f%%)\n', valid_count, n_samples, 100*valid_count/n_samples);

    % Compute convex hull volume
    try
        [~, volume] = convhull(points(:,1), points(:,2), points(:,3));
    catch
        warning('workspace_scan:ConvexHullFailed', 'Convex hull computation failed');
        volume = 0;
    end

    % Analyze tray plane coverage
    [grid_reachable, tray_coverage] = analyze_tray_coverage(points, C);

    % Store results
    results.points = points;
    results.volume = volume;
    results.tray_coverage = tray_coverage;
    results.grid_reachable = grid_reachable;
    results.samples = n_samples;
    results.valid_samples = valid_count;

    % Compute detailed metrics
    metrics.volume = volume;
    metrics.tray_coverage = tray_coverage;
    metrics.reachable_cells = sum(grid_reachable(:));
    metrics.total_cells = C.gridNx * C.gridNy;
    metrics.min_x = min(points(:,1));
    metrics.max_x = max(points(:,1));
    metrics.min_y = min(points(:,2));
    metrics.max_y = max(points(:,2));
    metrics.min_z = min(points(:,3));
    metrics.max_z = max(points(:,3));

    fprintf('\n=== Workspace Metrics ===\n');
    fprintf('Convex hull volume: %.6f m³\n', volume);
    fprintf('Tray coverage: %.1f%% (%d/%d cells)\n', ...
            tray_coverage*100, metrics.reachable_cells, metrics.total_cells);
    fprintf('X range: [%.3f, %.3f] m\n', metrics.min_x, metrics.max_x);
    fprintf('Y range: [%.3f, %.3f] m\n', metrics.min_y, metrics.max_y);
    fprintf('Z range: [%.3f, %.3f] m\n', metrics.min_z, metrics.max_z);

    % Visualization
    if visualize
        visualize_workspace(points, grid_reachable, C);
    end

end


function [grid_reachable, coverage] = analyze_tray_coverage(points, C)
% Analyze which tray grid cells are reachable with safety margin

    % Generate tray grid positions
    [grid_x, grid_y] = generate_tray_grid(C);
    n_cells = numel(grid_x);

    % Check reachability for each cell
    grid_reachable = false(C.gridNy, C.gridNx);

    % Filter points near tray height
    z_tolerance = 0.05;  % ±5cm around tray height
    tray_points = points(abs(points(:,3) - C.z_tray) < z_tolerance, :);

    for row = 1:C.gridNy
        for col = 1:C.gridNx
            target_x = grid_x(row, col);
            target_y = grid_y(row, col);

            % Check if any point is within safety margin
            distances = sqrt((tray_points(:,1) - target_x).^2 + ...
                           (tray_points(:,2) - target_y).^2);

            if any(distances < C.clearance)
                grid_reachable(row, col) = true;
            end
        end
    end

    coverage = sum(grid_reachable(:)) / n_cells;

end


function [grid_x, grid_y] = generate_tray_grid(C)
% Generate tray grid cell positions

    x_start = C.grid_x_offset - (C.gridNx-1)*C.gridDx/2;
    y_start = C.grid_y_offset - (C.gridNy-1)*C.gridDy/2;

    x_vec = x_start + (0:C.gridNx-1) * C.gridDx;
    y_vec = y_start + (0:C.gridNy-1) * C.gridDy;

    [grid_x, grid_y] = meshgrid(x_vec, y_vec);

end


function visualize_workspace(points, grid_reachable, C)
% Generate workspace visualization plots

    utils_plotting('setup');

    % 1. 3D scatter plot of reachable workspace
    fig1 = figure('Position', [100, 100, 800, 600]);
    scatter3(points(:,1), points(:,2), points(:,3), 1, points(:,3), 'filled', 'MarkerFaceAlpha', 0.1);
    utils_plotting('style_3d');
    title('Reachable Workspace (3D)', 'FontSize', 14, 'FontWeight', 'bold');
    colorbar;
    ylabel(colorbar, 'Z [m]');
    utils_plotting('save', 'phase1_foundation/figs/workspace_3d.png');

    % 2. 2D heatmap at tray height
    fig2 = figure('Position', [100, 100, 800, 600]);
    z_tolerance = 0.02;
    tray_points = points(abs(points(:,3) - C.z_tray) < z_tolerance, :);

    % Create 2D histogram
    x_edges = linspace(min(points(:,1)), max(points(:,1)), 50);
    y_edges = linspace(min(points(:,2)), max(points(:,2)), 50);
    heatmap_counts = histcounts2(tray_points(:,1), tray_points(:,2), x_edges, y_edges);

    imagesc(x_edges, y_edges, heatmap_counts');
    axis xy;
    colormap(jet);
    colorbar;
    hold on;

    % Overlay tray grid
    [grid_x, grid_y] = generate_tray_grid(C);
    for row = 1:C.gridNy
        for col = 1:C.gridNx
            if grid_reachable(row, col)
                plot(grid_x(row, col), grid_y(row, col), 'go', 'MarkerSize', 8, 'LineWidth', 2);
            else
                plot(grid_x(row, col), grid_y(row, col), 'rx', 'MarkerSize', 8, 'LineWidth', 2);
            end
        end
    end

    xlabel('X [m]', 'FontSize', C.font_size);
    ylabel('Y [m]', 'FontSize', C.font_size);
    title(sprintf('Workspace at Tray Height (z=%.2fm) - Coverage: %.1f%%', ...
                  C.z_tray, 100*sum(grid_reachable(:))/numel(grid_reachable)), ...
          'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    axis equal;
    legend({'Density', 'Reachable', 'Unreachable'}, 'Location', 'best');
    utils_plotting('save', 'phase1_foundation/figs/workspace_tray_plane.png');

    fprintf('\nWorkspace plots saved to phase1_foundation/figs/\n');

end

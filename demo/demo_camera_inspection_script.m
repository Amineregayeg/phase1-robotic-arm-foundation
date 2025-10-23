function stats = demo_camera_inspection_script()
% DEMO_CAMERA_INSPECTION_SCRIPT - Hydroponic tray computer vision inspection
%
% Demonstrates robotic arm autonomously navigating to inspection points above
% a hydroponic tray to position an end-effector-mounted camera for plant
% health monitoring. Tests first 6 inspection points with complete trajectories.
%
% Application: Computer Vision-based Plant Monitoring
% - Position camera at consistent height above each plant
% - Maintain downward orientation for overhead imaging
% - Navigate smoothly between inspection points
% - Collect imaging statistics (position accuracy, stability)
%
% Returns:
%   stats - Struct with inspection demonstration statistics
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║  HYDROPONIC CAMERA INSPECTION DEMONSTRATION (6 PLANTS)     ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');

    % Load configuration
    C = config();

    % Add source path
    addpath(fullfile(pwd, 'phase1_foundation', 'src'));

    % Initialize statistics
    stats = struct();
    stats.n_attempts = 0;
    stats.n_success = 0;
    stats.ik_iters = [];
    stats.cond_numbers = [];
    stats.clearances = [];
    stats.errors = [];

    % Generate inspection grid (camera imaging points above each plant)
    [inspection_grid_x, inspection_grid_y] = generate_inspection_grid(C);

    % Demo first 6 inspection points
    n_demo_points = 6;

    % Setup visualization
    utils_plotting('setup');
    fig = figure('Position', [100, 100, 1200, 800]);

    % Process each inspection point
    for point_idx = 1:n_demo_points
        fprintf('[Plant %d/%d] Inspecting...\n', point_idx, n_demo_points);

        % Get inspection point position
        [row, col] = ind2sub([C.gridNy, C.gridNx], point_idx);
        inspect_x = inspection_grid_x(row, col);
        inspect_y = inspection_grid_y(row, col);

        % Define waypoints: transit → imaging position → capture → return to transit
        waypoints = [
            inspect_x,  inspect_y,  C.z_tray + C.z_transit, 0;    % 1. Approach (transit height)
            inspect_x,  inspect_y,  C.z_tray,               0;    % 2. Imaging position (camera focus)
            inspect_x,  inspect_y,  C.z_tray,               0;    % 3. Hold for capture (simulated)
            inspect_x,  inspect_y,  C.z_tray + C.z_transit, 0     % 4. Retract to transit height
        ];

        % Attempt IK for each waypoint
        cell_success = true;
        q_prev = zeros(C.n, 1);  % Start from home

        for wp_idx = 1:size(waypoints, 1)
            target = waypoints(wp_idx, :)';

            stats.n_attempts = stats.n_attempts + 1;

            [q_sol, status, iters, final_error, sing_events] = ik_dls(target, q_prev, C);

            % Compute singularity metrics
            [~, cond_J, ~, ~] = singularity_metrics(q_sol, C);

            % Store statistics
            stats.ik_iters = [stats.ik_iters; iters];
            stats.cond_numbers = [stats.cond_numbers; cond_J];
            stats.errors = [stats.errors; final_error(1)];

            if strcmp(status, 'success')
                stats.n_success = stats.n_success + 1;
                q_prev = q_sol;  % Update for next waypoint

                fprintf('  WP%d: ✓ Success (iters=%d, cond=%.1f, err=%.4fmm)\n', ...
                        wp_idx, iters, cond_J, final_error(1)*1000);
            else
                cell_success = false;
                fprintf('  WP%d: ✗ Failed (%s)\n', wp_idx, status);
                break;
            end
        end

        if cell_success
            fprintf('  Plant %d: ✓ INSPECTION COMPLETE\n\n', point_idx);
        else
            fprintf('  Plant %d: ✗ INSPECTION FAILED\n\n', point_idx);
        end

        % Visualize arm configuration at imaging position
        if point_idx <= 6
            subplot(2, 3, point_idx);
            visualize_arm_config(q_prev, C);
            title(sprintf('Plant %d: (%d,%d) - Camera Position', point_idx, row, col), ...
                  'FontSize', 12, 'FontWeight', 'bold');
        end
    end

    % Compute overall statistics
    stats.success_rate = stats.n_success / stats.n_attempts;
    stats.mean_iters = mean(stats.ik_iters);
    stats.max_iters = max(stats.ik_iters);
    stats.mean_cond = mean(stats.cond_numbers);
    stats.max_cond = max(stats.cond_numbers);
    stats.mean_error = mean(stats.errors);
    stats.max_error = max(stats.errors);

    % Save visualization
    utils_plotting('save', 'phase1_foundation/demo/demo_screenshots/camera_positions.png');

    % Print summary
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║              INSPECTION DEMONSTRATION SUMMARY              ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('Inspection Success Rate: %.1f%% (%d/%d waypoints)\n', stats.success_rate*100, ...
            stats.n_success, stats.n_attempts);
    fprintf('IK Iterations: mean=%.1f, max=%d\n', stats.mean_iters, stats.max_iters);
    fprintf('Camera Position Accuracy: mean=%.4fmm, max=%.4fmm\n', ...
            stats.mean_error*1000, stats.max_error*1000);
    fprintf('Manipulability: cond(J) mean=%.1f, max=%.1f\n', stats.mean_cond, stats.max_cond);
    fprintf('\n');

    % Export statistics to CSV
    stats_table = table(...
        stats.ik_iters, ...
        stats.cond_numbers, ...
        stats.errors * 1000, ...  % Convert to mm
        'VariableNames', {'IK_Iterations', 'Condition_Number', 'Position_Error_mm'});

    writetable(stats_table, 'phase1_foundation/demo/demo_statistics.csv');
    fprintf('Statistics exported to: phase1_foundation/demo/demo_statistics.csv\n\n');

end


function [grid_x, grid_y] = generate_inspection_grid(C)
% Generate inspection grid positions (camera imaging points above plants)
%
% Returns grid of (x,y) coordinates where camera should be positioned
% for overhead imaging of each plant in the hydroponic tray

    x_start = C.grid_x_offset - (C.gridNx-1)*C.gridDx/2;
    y_start = C.grid_y_offset - (C.gridNy-1)*C.gridDy/2;

    x_vec = x_start + (0:C.gridNx-1) * C.gridDx;
    y_vec = y_start + (0:C.gridNy-1) * C.gridDy;

    [grid_x, grid_y] = meshgrid(x_vec, y_vec);

end


function visualize_arm_config(q, C)
% Visualize arm configuration

    % Compute FK for all links
    [T, Ti, ~] = fk(q, C);

    % Extract link positions
    positions = zeros(C.n+1, 3);
    positions(1, :) = [0, 0, 0];  % Base

    for i = 1:C.n
        T_i = Ti(:, :, i);
        positions(i+1, :) = T_i(1:3, 4)';
    end

    % Plot arm
    plot3(positions(:, 1), positions(:, 2), positions(:, 3), ...
          'b-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    hold on;

    % Plot end-effector
    ee_pos = T(1:3, 4);
    plot3(ee_pos(1), ee_pos(2), ee_pos(3), ...
          'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');

    % Plot tray plane
    [X, Y] = meshgrid(linspace(-0.1, 0.4, 10), linspace(-0.2, 0.4, 10));
    Z = ones(size(X)) * C.z_tray;
    surf(X, Y, Z, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'g');

    grid on;
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    view(3);
    xlim([-0.1, 0.4]);
    ylim([-0.2, 0.4]);
    zlim([0, 0.5]);

end

function results = test_trajectory_function()
% TEST_TRAJECTORY_FUNCTION - Unit tests for trajectory planning
%
% Tests trajectory continuity, limits, and clearance constraints
%
% Returns:
%   results - Struct with test metrics and pass/fail status
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    fprintf('  Testing trajectory planning and constraints...\n');

    % Load configuration
    C = config();

    % Define test pick-place-retract sequence
    pick_pose = [0.20; 0.05; C.z_tray; 0];           % Pick at tray
    lift_pose = [0.20; 0.05; C.z_tray + C.z_lift; 0]; % Lift
    place_pose = [0.20; 0.30; C.z_tray; 0];          % Place at offset location
    retract_pose = [0.20; 0.30; C.z_tray + C.z_lift; 0]; % Retract

    % Plan multi-segment trajectory
    segments = {
        {pick_pose, lift_pose, 1.5, 'task_lspb'},
        {lift_pose, place_pose, 2.0, 'task_lspb'},
        {place_pose, retract_pose, 1.5, 'task_lspb'}
    };

    % Initialize metrics
    limit_violations = 0;
    continuity_violations = 0;
    jerk_violations = 0;
    clearance_violations = 0;
    min_clearance = inf;

    % Plan each segment
    all_valid = true;
    for i = 1:length(segments)
        seg = segments{i};
        start_p = seg{1};
        goal_p = seg{2};
        Tf = seg{3};
        method = seg{4};

        fprintf('    Segment %d/%d: %s...\n', i, length(segments), method);

        [traj, valid, violations] = plan_trajectory(start_p, goal_p, Tf, method, C);

        if ~valid
            all_valid = false;
            if violations.limit
                limit_violations = limit_violations + 1;
            end
            if violations.continuity
                continuity_violations = continuity_violations + 1;
            end
            if violations.jerk
                jerk_violations = jerk_violations + 1;
            end
            if violations.clearance
                clearance_violations = clearance_violations + 1;
            end
        end

        min_clearance = min(min_clearance, traj.clearance);

        % Save trajectory plots for first segment
        if i == 1
            save_trajectory_plots(traj, C);
        end
    end

    % Check pass criteria
    pass_no_violations = all_valid;
    pass_clearance = min_clearance >= C.pass_clearance_min;

    all_passed = pass_no_violations && pass_clearance;

    % Store results
    results.all_passed = all_passed;
    results.limit_violations = limit_violations;
    results.continuity_violations = continuity_violations;
    results.jerk_violations = jerk_violations;
    results.clearance_violations = clearance_violations;
    results.min_clearance = min_clearance;
    results.pass_no_violations = pass_no_violations;
    results.pass_clearance = pass_clearance;

    % Print detailed results
    fprintf('    Limit Violations: %d [%s]\n', limit_violations, pass_str(limit_violations == 0));
    fprintf('    Continuity Violations: %d [%s]\n', continuity_violations, pass_str(continuity_violations == 0));
    fprintf('    Jerk Violations: %d [%s]\n', jerk_violations, pass_str(jerk_violations == 0));
    fprintf('    Clearance Violations: %d [%s]\n', clearance_violations, pass_str(clearance_violations == 0));
    fprintf('    Min Clearance: %.4f m (threshold: ≥ %.4f m) [%s]\n', ...
            min_clearance, C.pass_clearance_min, pass_str(pass_clearance));

end

function save_trajectory_plots(traj, C)
% Save trajectory visualization plots

    utils_plotting('setup');

    % Joint position profiles
    fig = figure('Position', [100, 100, 1000, 600]);

    subplot(3, 1, 1);
    plot(traj.t, rad2deg(traj.q), 'LineWidth', 1.5);
    grid on;
    ylabel('Position [deg]', 'FontSize', C.font_size);
    title('Joint Trajectories', 'FontSize', 14, 'FontWeight', 'bold');
    legend(arrayfun(@(i) sprintf('q_%d', i), 1:C.n, 'UniformOutput', false), ...
           'Location', 'eastoutside');

    subplot(3, 1, 2);
    plot(traj.t, rad2deg(traj.qd), 'LineWidth', 1.5);
    grid on;
    ylabel('Velocity [deg/s]', 'FontSize', C.font_size);

    subplot(3, 1, 3);
    plot(traj.t, rad2deg(traj.qdd), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]', 'FontSize', C.font_size);
    ylabel('Acceleration [deg/s²]', 'FontSize', C.font_size);

    utils_plotting('save', 'phase1_foundation/figs/trajectory_profiles.png');

    % End-effector path
    fig2 = figure('Position', [100, 100, 800, 600]);
    ee_positions = squeeze(traj.pose(1:3, 4, :))';

    plot3(ee_positions(:, 1), ee_positions(:, 2), ee_positions(:, 3), ...
          'b-', 'LineWidth', 2);
    hold on;
    plot3(ee_positions(1, 1), ee_positions(1, 2), ee_positions(1, 3), ...
          'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot3(ee_positions(end, 1), ee_positions(end, 2), ee_positions(end, 3), ...
          'ro', 'MarkerSize', 10, 'LineWidth', 2);

    utils_plotting('style_3d');
    title('End-Effector Path', 'FontSize', 14, 'FontWeight', 'bold');
    legend({'Path', 'Start', 'Goal'}, 'Location', 'best');

    utils_plotting('save', 'phase1_foundation/figs/ee_path.png');

    fprintf('    Trajectory plots saved to phase1_foundation/figs/\n');

end

function s = pass_str(passed)
    if passed
        s = '✓ PASS';
    else
        s = '✗ FAIL';
    end
end

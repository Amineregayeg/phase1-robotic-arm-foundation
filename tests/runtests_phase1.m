function exit_code = runtests_phase1()
% RUNTESTS_PHASE1 - Execute all Phase-1 validation tests
%
% Runs all unit tests and reports PASS/FAIL status for CI/grading.
%
% Returns:
%   exit_code - 0 if all tests pass, 1 if any test fails
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║          PHASE-1 UNIT TEST SUITE EXECUTION                ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');

    % Add source path
    addpath(fullfile(pwd, 'phase1_foundation', 'src'));

    % Initialize results
    all_passed = true;
    test_results = struct();

    % Test 1: FK/IK Round-trip
    fprintf('[1/3] Running FK/IK tests...\n');
    try
        results_fk_ik = test_fk_ik_function();
        test_results.fk_ik = results_fk_ik;

        if results_fk_ik.all_passed
            fprintf('  ✓ FK/IK tests PASSED\n');
        else
            fprintf('  ✗ FK/IK tests FAILED\n');
            all_passed = false;
        end
    catch ME
        fprintf('  ✗ FK/IK tests CRASHED: %s\n', ME.message);
        all_passed = false;
        test_results.fk_ik.all_passed = false;
    end
    fprintf('\n');

    % Test 2: Workspace coverage
    fprintf('[2/3] Running Workspace tests...\n');
    try
        results_workspace = test_workspace_function();
        test_results.workspace = results_workspace;

        if results_workspace.all_passed
            fprintf('  ✓ Workspace tests PASSED\n');
        else
            fprintf('  ✗ Workspace tests FAILED\n');
            all_passed = false;
        end
    catch ME
        fprintf('  ✗ Workspace tests CRASHED: %s\n', ME.message);
        all_passed = false;
        test_results.workspace.all_passed = false;
    end
    fprintf('\n');

    % Test 3: Trajectory planning
    fprintf('[3/3] Running Trajectory tests...\n');
    try
        results_traj = test_trajectory_function();
        test_results.trajectory = results_traj;

        if results_traj.all_passed
            fprintf('  ✓ Trajectory tests PASSED\n');
        else
            fprintf('  ✗ Trajectory tests FAILED\n');
            all_passed = false;
        end
    catch ME
        fprintf('  ✗ Trajectory tests CRASHED: %s\n', ME.message);
        all_passed = false;
        test_results.trajectory.all_passed = false;
    end
    fprintf('\n');

    % Final summary
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    if all_passed
        fprintf('║                   ALL TESTS PASSED ✓                       ║\n');
        exit_code = 0;
    else
        fprintf('║                   SOME TESTS FAILED ✗                      ║\n');
        exit_code = 1;
    end
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');

    % Display detailed metrics
    if isfield(test_results, 'fk_ik') && test_results.fk_ik.all_passed
        fprintf('FK/IK Metrics:\n');
        fprintf('  Max position error: %.6f m\n', test_results.fk_ik.max_pos_error);
        fprintf('  Max yaw error: %.4f deg\n', rad2deg(test_results.fk_ik.max_yaw_error));
        fprintf('  Mean iterations: %.1f\n', test_results.fk_ik.mean_iters);
    end

    if isfield(test_results, 'workspace') && test_results.workspace.all_passed
        fprintf('Workspace Metrics:\n');
        fprintf('  Tray coverage: %.1f%%\n', test_results.workspace.coverage * 100);
        fprintf('  Reachable cells: %d/%d\n', test_results.workspace.reachable_cells, ...
                test_results.workspace.total_cells);
    end

    if isfield(test_results, 'trajectory') && test_results.trajectory.all_passed
        fprintf('Trajectory Metrics:\n');
        fprintf('  Min clearance: %.4f m\n', test_results.trajectory.min_clearance);
        fprintf('  Limit violations: %d\n', test_results.trajectory.limit_violations);
    end

    fprintf('\n');

end

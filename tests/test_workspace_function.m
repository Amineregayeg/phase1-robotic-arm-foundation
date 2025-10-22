function results = test_workspace_function()
% TEST_WORKSPACE_FUNCTION - Unit tests for workspace coverage
%
% Tests workspace reachability and tray grid coverage
%
% Returns:
%   results - Struct with test metrics and pass/fail status
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    fprintf('  Computing workspace coverage (this may take a minute)...\n');

    % Load configuration
    C = config();

    % Run workspace scan (without visualization to save time)
    [ws_results, metrics] = workspace_scan(C, false);

    % Extract metrics
    coverage = ws_results.tray_coverage;
    reachable_cells = metrics.reachable_cells;
    total_cells = metrics.total_cells;
    volume = ws_results.volume;

    % Check pass criteria
    pass_coverage = coverage >= C.pass_coverage;

    all_passed = pass_coverage;

    % Store results
    results.all_passed = all_passed;
    results.coverage = coverage;
    results.reachable_cells = reachable_cells;
    results.total_cells = total_cells;
    results.volume = volume;
    results.pass_coverage = pass_coverage;

    % Print detailed results
    fprintf('    Tray Coverage: %.1f%% (%d/%d cells) (threshold: ≥ %.1f%%) [%s]\n', ...
            coverage*100, reachable_cells, total_cells, ...
            C.pass_coverage*100, pass_str(pass_coverage));
    fprintf('    Workspace Volume: %.6f m³\n', volume);

    % Export coverage grid to CSV for documentation
    csv_file = 'phase1_foundation/figs/tray_coverage.csv';
    writematrix(double(ws_results.grid_reachable), csv_file);
    fprintf('    Coverage grid saved to: %s\n', csv_file);

end

function s = pass_str(passed)
    if passed
        s = '✓ PASS';
    else
        s = '✗ FAIL';
    end
end

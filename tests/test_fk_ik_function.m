function results = test_fk_ik_function()
% TEST_FK_IK_FUNCTION - Unit tests for forward and inverse kinematics
%
% Tests FK/IK round-trip accuracy on reachable targets
%
% Returns:
%   results - Struct with test metrics and pass/fail status
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    fprintf('  Testing FK orthonormality and IK convergence...\n');

    % Load configuration
    C = config();

    % Set random seed
    rng(C.seed);

    % Test parameters
    n_tests = 200;
    max_pos_error = 0;
    max_yaw_error = 0;
    total_iters = 0;
    failures = 0;
    orthonorm_errors = [];

    % Generate random reachable configurations
    for i = 1:n_tests
        % Random joint configuration within limits
        q_rand = zeros(C.n, 1);
        for j = 1:C.n
            q_rand(j) = C.qmin(j) + rand() * (C.qmax(j) - C.qmin(j));
        end

        % Test 1: FK orthonormality
        [T, Ti, valid] = fk(q_rand, C);

        if ~valid
            failures = failures + 1;
            continue;
        end

        R = T(1:3, 1:3);
        orthonorm_error = norm(R' * R - eye(3), 'fro');
        orthonorm_errors = [orthonorm_errors; orthonorm_error];

        % Test 2: IK round-trip
        pos_target = T(1:3, 4);
        yaw_target = atan2(T(2, 1), T(1, 1));
        target = [pos_target; yaw_target];

        % Initial guess: perturb slightly from solution
        q0 = q_rand + 0.1 * randn(C.n, 1);
        q0 = enforce_limits(q0, C);

        [q_sol, status, iters, final_error, ~] = ik_dls(target, q0, C);

        if strcmp(status, 'success')
            % Verify round-trip
            [T_sol, ~, ~] = fk(q_sol, C);
            pos_sol = T_sol(1:3, 4);
            yaw_sol = atan2(T_sol(2, 1), T_sol(1, 1));

            pos_error = norm(pos_target - pos_sol);
            yaw_error = abs(wrapToPi(yaw_target - yaw_sol));

            max_pos_error = max(max_pos_error, pos_error);
            max_yaw_error = max(max_yaw_error, yaw_error);
            total_iters = total_iters + iters;
        else
            failures = failures + 1;
        end

        if mod(i, 50) == 0
            fprintf('    Progress: %d/%d tests\n', i, n_tests);
        end
    end

    % Compute metrics
    mean_iters = total_iters / (n_tests - failures);
    success_rate = (n_tests - failures) / n_tests;
    max_orthonorm_error = max(orthonorm_errors);

    % Check pass criteria
    pass_orthonorm = max_orthonorm_error < C.pass_fk_orthonorm;
    pass_ik_pos = max_pos_error < C.pass_ik_pos_error;
    pass_ik_yaw = max_yaw_error < C.pass_ik_yaw_error;
    pass_ik_iters = mean_iters < C.pass_ik_max_iters;
    pass_success_rate = success_rate >= 0.95;  % 95% success rate

    all_passed = pass_orthonorm && pass_ik_pos && pass_ik_yaw && pass_ik_iters && pass_success_rate;

    % Store results
    results.all_passed = all_passed;
    results.max_pos_error = max_pos_error;
    results.max_yaw_error = max_yaw_error;
    results.mean_iters = mean_iters;
    results.success_rate = success_rate;
    results.max_orthonorm_error = max_orthonorm_error;
    results.pass_orthonorm = pass_orthonorm;
    results.pass_ik_pos = pass_ik_pos;
    results.pass_ik_yaw = pass_ik_yaw;
    results.pass_ik_iters = pass_ik_iters;

    % Print detailed results
    fprintf('    FK Orthonormality: max error = %.2e (threshold: %.2e) [%s]\n', ...
            max_orthonorm_error, C.pass_fk_orthonorm, pass_str(pass_orthonorm));
    fprintf('    IK Position Error: max = %.4f mm (threshold: %.4f mm) [%s]\n', ...
            max_pos_error*1000, C.pass_ik_pos_error*1000, pass_str(pass_ik_pos));
    fprintf('    IK Yaw Error: max = %.4f deg (threshold: %.4f deg) [%s]\n', ...
            rad2deg(max_yaw_error), rad2deg(C.pass_ik_yaw_error), pass_str(pass_ik_yaw));
    fprintf('    IK Iterations: mean = %.1f (threshold: < %d) [%s]\n', ...
            mean_iters, C.pass_ik_max_iters, pass_str(pass_ik_iters));
    fprintf('    Success Rate: %.1f%% (threshold: ≥ 95%%) [%s]\n', ...
            success_rate*100, pass_str(pass_success_rate));

end

function s = pass_str(passed)
    if passed
        s = '✓ PASS';
    else
        s = '✗ FAIL';
    end
end

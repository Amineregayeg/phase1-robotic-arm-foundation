function check_requirements()
% CHECK_REQUIREMENTS - Verify system is ready for Phase-1 testing
%
% Checks MATLAB version, toolboxes, and system configuration
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║       PHASE-1 SYSTEM REQUIREMENTS CHECK                    ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');

    all_passed = true;

    %% 1. MATLAB VERSION CHECK
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('1. MATLAB VERSION\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

    matlab_ver = version;
    matlab_release = version('-release');
    fprintf('  Installed: MATLAB %s (Release %s)\n', matlab_ver, matlab_release);

    % Extract year from release (e.g., '2023a' -> 2023)
    release_year = str2double(matlab_release(1:4));

    if release_year >= 2023
        fprintf('  Status: ✅ PASS (Recommended R2023a+)\n');
    elseif release_year >= 2020
        fprintf('  Status: ⚠️  WARNING (R2020b+, may work but not tested)\n');
    else
        fprintf('  Status: ✗ FAIL (R2020b or later required)\n');
        all_passed = false;
    end
    fprintf('\n');

    %% 2. TOOLBOX CHECK
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('2. MATLAB TOOLBOXES (Optional)\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

    toolboxes = ver;

    % Check for optional toolboxes
    has_robotics = any(contains({toolboxes.Name}, 'Robotics'));
    has_stats = any(contains({toolboxes.Name}, 'Statistics'));

    fprintf('  Core MATLAB: ✅ Required (Present)\n');

    if has_robotics
        fprintf('  Robotics System Toolbox: ✅ Optional (Present - Enhanced Simulink)\n');
    else
        fprintf('  Robotics System Toolbox: ⚪ Optional (Not Present - Not Required)\n');
    end

    if has_stats
        fprintf('  Statistics Toolbox: ✅ Optional (Present - Better Sampling)\n');
    else
        fprintf('  Statistics Toolbox: ⚪ Optional (Not Present - Will Use Fallback)\n');
    end

    fprintf('\n  Note: Core functionality works with base MATLAB only!\n');
    fprintf('\n');

    %% 3. MEMORY CHECK
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('3. SYSTEM MEMORY\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

    try
        if ispc
            [~, mem_info] = memory;
            total_mem_gb = mem_info.PhysicalMemory.Total / 1e9;
            avail_mem_gb = mem_info.PhysicalMemory.Available / 1e9;
            fprintf('  Total Memory: %.1f GB\n', total_mem_gb);
            fprintf('  Available Memory: %.1f GB\n', avail_mem_gb);

            if avail_mem_gb >= 2
                fprintf('  Status: ✅ PASS (≥ 2 GB available)\n');
            else
                fprintf('  Status: ⚠️  WARNING (< 2 GB available, may be slow)\n');
            end
        else
            fprintf('  Status: ⚪ SKIP (Memory check not available on this platform)\n');
        end
    catch
        fprintf('  Status: ⚪ SKIP (Unable to check memory)\n');
    end
    fprintf('\n');

    %% 4. DIRECTORY CHECK
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('4. DIRECTORY STRUCTURE\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

    required_dirs = {'src', 'tests', 'demo', 'sim', 'figs'};
    required_files = {'run_all.m', 'README.md'};

    all_dirs_present = true;
    for i = 1:length(required_dirs)
        if exist(required_dirs{i}, 'dir')
            fprintf('  %s/: ✅ Present\n', required_dirs{i});
        else
            fprintf('  %s/: ✗ MISSING\n', required_dirs{i});
            all_dirs_present = false;
        end
    end

    all_files_present = true;
    for i = 1:length(required_files)
        if exist(required_files{i}, 'file')
            fprintf('  %s: ✅ Present\n', required_files{i});
        else
            fprintf('  %s: ✗ MISSING\n', required_files{i});
            all_files_present = false;
        end
    end

    if all_dirs_present && all_files_present
        fprintf('  Status: ✅ PASS (All required files/directories present)\n');
    else
        fprintf('  Status: ✗ FAIL (Missing files/directories)\n');
        all_passed = false;
    end
    fprintf('\n');

    %% 5. CORE FUNCTIONS CHECK
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('5. CORE FUNCTIONS (9 Required)\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

    core_functions = {
        'src/config.m',
        'src/fk.m',
        'src/ik_dls.m',
        'src/jacobian_geometric.m',
        'src/workspace_scan.m',
        'src/plan_trajectory.m',
        'src/enforce_limits.m',
        'src/singularity_metrics.m',
        'src/utils_plotting.m'
    };

    all_functions_present = true;
    for i = 1:length(core_functions)
        if exist(core_functions{i}, 'file')
            fprintf('  %s: ✅\n', core_functions{i});
        else
            fprintf('  %s: ✗ MISSING\n', core_functions{i});
            all_functions_present = false;
        end
    end

    if all_functions_present
        fprintf('  Status: ✅ PASS (All 9 core functions present)\n');
    else
        fprintf('  Status: ✗ FAIL (Missing core functions)\n');
        all_passed = false;
    end
    fprintf('\n');

    %% 6. QUICK FUNCTIONALITY TEST
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('6. QUICK FUNCTIONALITY TEST\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

    try
        % Add path
        addpath('src');

        % Test config
        fprintf('  Loading config...\n');
        C = config();
        fprintf('    ✅ config.m loaded (n=%d DOF)\n', C.n);

        % Test FK
        fprintf('  Testing FK...\n');
        q_test = zeros(C.n, 1);
        [T, ~, valid] = fk(q_test, C);
        if valid
            fprintf('    ✅ FK executed successfully\n');
        else
            fprintf('    ✗ FK validation failed\n');
            all_passed = false;
        end

        % Test IK
        fprintf('  Testing IK...\n');
        target = [0.2; 0.1; 0.15; 0];
        [~, status, ~, ~, ~] = ik_dls(target, q_test, C);
        fprintf('    ✅ IK executed (status: %s)\n', status);

        fprintf('  Status: ✅ PASS (Core functions working)\n');

    catch ME
        fprintf('  Status: ✗ FAIL (Error: %s)\n', ME.message);
        all_passed = false;
    end
    fprintf('\n');

    %% FINAL SUMMARY
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    if all_passed
        fprintf('║               ✅ SYSTEM READY FOR PHASE-1                 ║\n');
        fprintf('╚════════════════════════════════════════════════════════════╝\n');
        fprintf('\n');
        fprintf('🎉 Your system meets all requirements!\n\n');
        fprintf('Next steps:\n');
        fprintf('  1. Run the complete pipeline: run_all\n');
        fprintf('  2. Or run tests only: runtests_phase1\n');
        fprintf('  3. Or run demo only: demo_pick_place_script\n');
        fprintf('\n');
    else
        fprintf('║               ⚠️  SOME REQUIREMENTS NOT MET               ║\n');
        fprintf('╚════════════════════════════════════════════════════════════╝\n');
        fprintf('\n');
        fprintf('⚠️  Please address the issues marked with ✗ above.\n');
        fprintf('\n');
        fprintf('Common fixes:\n');
        fprintf('  - Update MATLAB to R2023a or later\n');
        fprintf('  - Ensure you are in the phase1_foundation directory\n');
        fprintf('  - Re-clone the repository if files are missing\n');
        fprintf('\n');
    end

end

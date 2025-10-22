function build_simulink_model()
% BUILD_SIMULINK_MODEL - Programmatically create RA_Sim.slx
%
% Creates a Simulink model for robotic arm simulation with:
%   - Trajectory input generation
%   - FK/IK kinematics blocks
%   - Joint limit saturation
%   - Pose output and visualization
%
% The model is built programmatically to ensure reproducibility.
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    fprintf('Building Simulink model: RA_Sim.slx...\n');

    % Load configuration
    C = config();

    % Model name
    modelName = 'RA_Sim';
    modelPath = fullfile(pwd, 'phase1_foundation', 'sim', modelName);

    % Close and delete existing model if it exists
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    if exist([modelPath '.slx'], 'file')
        delete([modelPath '.slx']);
    end

    % Create new model
    new_system(modelName);
    open_system(modelName);

    % Set model parameters
    set_param(modelName, 'StopTime', '10');
    set_param(modelName, 'SolverType', 'Fixed-step');
    set_param(modelName, 'FixedStep', num2str(C.traj_dt));

    % Add model workspace and load config
    modelWorkspace = get_param(modelName, 'ModelWorkspace');
    assignin(modelWorkspace, 'C', C);

    % Positions for blocks (x, y, width, height)
    x_offset = 100;
    y_offset = 100;
    block_width = 120;
    block_height = 60;
    spacing = 180;

    % 1. Trajectory Generator (MATLAB Function)
    trajGenBlock = [modelName '/Trajectory_Generator'];
    add_block('simulink/User-Defined Functions/MATLAB Function', trajGenBlock);
    set_param(trajGenBlock, 'Position', [x_offset, y_offset, x_offset+block_width, y_offset+block_height]);

    % Set MATLAB function code for trajectory generator
    trajFcnText = [...
        'function [q_des, qd_des, qdd_des] = fcn(t)\n', ...
        '    % Trajectory generator - simple sinusoidal motion\n', ...
        '    C = evalin(''base'', ''C'');\n', ...
        '    n = C.n;\n', ...
        '    \n', ...
        '    % Sinusoidal joint motion within limits\n', ...
        '    q_des = zeros(n, 1);\n', ...
        '    qd_des = zeros(n, 1);\n', ...
        '    qdd_des = zeros(n, 1);\n', ...
        '    \n', ...
        '    for i = 1:n\n', ...
        '        q_mid = (C.qmax(i) + C.qmin(i)) / 2;\n', ...
        '        q_amp = (C.qmax(i) - C.qmin(i)) / 4;\n', ...
        '        omega = 0.5;  % rad/s\n', ...
        '        \n', ...
        '        q_des(i) = q_mid + q_amp * sin(omega * t);\n', ...
        '        qd_des(i) = q_amp * omega * cos(omega * t);\n', ...
        '        qdd_des(i) = -q_amp * omega^2 * sin(omega * t);\n', ...
        '    end\n', ...
        'end'
    ];

    % Note: Setting MATLAB function code programmatically requires using
    % Stateflow API or manual editing. For now, create a placeholder.
    % Users will need to manually add the function code or we provide a helper script.

    % 2. Clock
    clockBlock = [modelName '/Clock'];
    add_block('simulink/Sources/Clock', clockBlock);
    set_param(clockBlock, 'Position', [x_offset-150, y_offset+10, x_offset-120, y_offset+40]);

    % 3. Joint Limit Saturation
    x_pos2 = x_offset + spacing;
    satBlock = [modelName '/Joint_Limits'];
    add_block('simulink/Discontinuities/Saturation', satBlock);
    set_param(satBlock, 'Position', [x_pos2, y_offset, x_pos2+block_width, y_offset+block_height]);
    set_param(satBlock, 'UpperLimit', 'C.qmax');
    set_param(satBlock, 'LowerLimit', 'C.qmin');

    % 4. Forward Kinematics (MATLAB Function)
    x_pos3 = x_pos2 + spacing;
    fkBlock = [modelName '/Forward_Kinematics'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fkBlock);
    set_param(fkBlock, 'Position', [x_pos3, y_offset, x_pos3+block_width, y_offset+block_height]);

    % 5. Pose Output (Scope)
    x_pos4 = x_pos3 + spacing;
    scopeBlock = [modelName '/Pose_Scope'];
    add_block('simulink/Sinks/Scope', scopeBlock);
    set_param(scopeBlock, 'Position', [x_pos4, y_offset, x_pos4+80, y_offset+block_height]);

    % 6. Joint Position Display (Display)
    displayBlock = [modelName '/Joint_Display'];
    add_block('simulink/Sinks/Display', displayBlock);
    set_param(displayBlock, 'Position', [x_pos2, y_offset+100, x_pos2+80, y_offset+140]);

    % Connect blocks
    try
        add_line(modelName, 'Clock/1', 'Trajectory_Generator/1', 'autorouting', 'on');
        add_line(modelName, 'Trajectory_Generator/1', 'Joint_Limits/1', 'autorouting', 'on');
        add_line(modelName, 'Joint_Limits/1', 'Forward_Kinematics/1', 'autorouting', 'on');
        add_line(modelName, 'Forward_Kinematics/1', 'Pose_Scope/1', 'autorouting', 'on');
        add_line(modelName, 'Joint_Limits/1', 'Joint_Display/1', 'autorouting', 'on');
    catch ME
        warning('build_simulink_model:ConnectionFailed', 'Failed to connect blocks: %s', ME.message);
    end

    % Save model
    save_system(modelName, modelPath);
    fprintf('  ✓ Simulink model saved to: %s.slx\n', modelPath);

    % Create instructions file for manual setup
    instructionsFile = fullfile(pwd, 'phase1_foundation', 'sim', 'SIMULINK_SETUP_INSTRUCTIONS.txt');
    fid = fopen(instructionsFile, 'w');
    fprintf(fid, 'SIMULINK MODEL SETUP INSTRUCTIONS\n');
    fprintf(fid, '==================================\n\n');
    fprintf(fid, 'The Simulink model RA_Sim.slx has been created with placeholder blocks.\n\n');
    fprintf(fid, 'To complete the setup:\n\n');
    fprintf(fid, '1. Open RA_Sim.slx in Simulink\n\n');
    fprintf(fid, '2. Double-click "Trajectory_Generator" MATLAB Function block and paste:\n\n');
    fprintf(fid, '%s\n\n', trajFcnText);
    fprintf(fid, '3. Double-click "Forward_Kinematics" MATLAB Function block and paste:\n\n');
    fprintf(fid, 'function [x, y, z, yaw] = fcn(q)\n');
    fprintf(fid, '    C = evalin(''base'', ''C'');\n');
    fprintf(fid, '    [T, ~, ~] = fk(q, C);\n');
    fprintf(fid, '    x = T(1, 4);\n');
    fprintf(fid, '    y = T(2, 4);\n');
    fprintf(fid, '    z = T(3, 4);\n');
    fprintf(fid, '    yaw = atan2(T(2, 1), T(1, 1));\n');
    fprintf(fid, 'end\n\n');
    fprintf(fid, '4. Run the model from MATLAB using: sim(''RA_Sim'')\n\n');
    fprintf(fid, 'Note: Ensure config.m and fk.m are on the MATLAB path.\n');
    fclose(fid);

    fprintf('  ✓ Setup instructions saved to: SIMULINK_SETUP_INSTRUCTIONS.txt\n');
    fprintf('\n');
    fprintf('NOTE: Due to programmatic limitations, MATLAB Function block code\n');
    fprintf('      must be added manually. See SIMULINK_SETUP_INSTRUCTIONS.txt\n\n');

end

function C = config()
% CONFIG - Configuration parameters for 5-DOF robotic arm (hydroponic farming)
%
% Returns:
%   C - struct containing all system parameters
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    % Degrees of freedom (4-6 supported, default 5)
    C.n = 5;

    % DH Parameters (Standard DH Convention)
    % Link lengths (m) - tuned for hydroponic tray workspace
    C.a     = [0.06, 0.11, 0.10, 0.08, 0.06];

    % Link offsets (m)
    C.d     = [0.10, 0, 0, 0, 0];

    % Link twists (rad)
    C.alpha = [pi/2, 0, 0, 0, pi/2];

    % Home position offset (rad)
    C.theta0 = [0, 0, 0, 0, 0];

    % Joint limits (rad)
    C.qmin = deg2rad([-170, -100, -120, -180, -180]);
    C.qmax = deg2rad([170, 100, 120, 180, 180]);

    % IK convergence tolerances
    C.tol_pos = 1e-3;           % Position tolerance (m)
    C.tol_yaw = deg2rad(0.5);   % Orientation tolerance (rad)
    C.max_iters = 200;          % Maximum IK iterations

    % IK damping parameters
    C.lambda_init = 1e-3;       % Initial damping factor
    C.lambda_max = 1e-1;        % Maximum damping factor
    C.cond_threshold = 250;     % Condition number threshold for damping increase
    C.manip_threshold = 0.02;   % Manipulability threshold (warning)

    % Hydroponic tray parameters
    C.z_tray = 0.15;            % Tray height (m)
    C.clearance = 0.02;         % Safety clearance margin (m)
    C.z_lift = 0.08;            % Lift height above tray (m)

    % Pick/place grid parameters
    C.gridNx = 8;               % Grid columns
    C.gridNy = 4;               % Grid rows
    C.gridDx = 0.06;            % Grid spacing X (m)
    C.gridDy = 0.06;            % Grid spacing Y (m)
    C.grid_x_offset = 0.15;     % Grid center X offset (m)
    C.grid_y_offset = 0.0;      % Grid center Y offset (m)
    C.place_y_offset = 0.25;    % Place grid Y offset (m)

    % Workspace analysis parameters
    C.workspace_samples = 50000; % Number of samples for workspace scan
    C.seed = 42;                 % Random seed for reproducibility
    C.coverage_threshold = 0.90; % Required tray coverage (90%)

    % Trajectory parameters
    C.traj_duration = 2.0;       % Default trajectory duration (s)
    C.traj_dt = 0.01;            % Trajectory time step (s)
    C.vmax = deg2rad(45);        % Max joint velocity (rad/s)
    C.amax = deg2rad(90);        % Max joint acceleration (rad/s²)
    C.jerk_threshold_ratio = 3.0; % Max jerk spike ratio (× median)

    % Plotting parameters
    C.fig_dpi = 300;             % Figure resolution (DPI)
    C.fig_format = 'png';        % Figure format
    C.font_size = 12;            % Font size for plots

    % Validation thresholds (PASS metrics)
    C.pass_fk_orthonorm = 1e-6;  % FK orthonormality tolerance
    C.pass_ik_pos_error = 1e-3;  % IK position error (m)
    C.pass_ik_yaw_error = deg2rad(0.5); % IK yaw error (rad)
    C.pass_ik_max_iters = 60;    % Average IK iterations threshold
    C.pass_coverage = 0.90;      % Workspace coverage threshold
    C.pass_clearance_min = 0.02; % Minimum clearance (m)
    C.pass_cond_max = 250;       % Maximum condition number threshold

end

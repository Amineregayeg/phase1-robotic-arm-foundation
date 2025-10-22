function [traj, valid, violations] = plan_trajectory(start_pose, goal_pose, Tf, method, C)
% PLAN_TRAJECTORY - Generate smooth trajectories with constraint checking
%
% Plans collision-free trajectories in joint or task space with continuity
% and limit validation.
%
% Inputs:
%   start_pose - Starting configuration:
%                [x,y,z,yaw] (4×1) for task-space OR
%                q_start (n×1) for joint-space
%   goal_pose  - Goal configuration (same format as start_pose)
%   Tf         - Trajectory duration (s)
%   method     - 'joint_cubic' or 'task_lspb' (default: 'joint_cubic')
%   C          - Configuration struct from config()
%
% Outputs:
%   traj       - Struct containing:
%       .t     - Time vector (K×1)
%       .q     - Joint positions (K×n)
%       .qd    - Joint velocities (K×n)
%       .qdd   - Joint accelerations (K×n)
%       .pose  - End-effector poses (4×4×K)
%       .clearance - Minimum clearance along trajectory (m)
%   valid      - Boolean, true if trajectory meets all constraints
%   violations - Struct with violation details
%
% Methods:
%   'joint_cubic' - Cubic polynomial in joint space (zero vel at endpoints)
%   'task_lspb'   - Linear segment with parabolic blends in task space
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    % Input validation
    if nargin < 5
        C = config();
    end
    if nargin < 4 || isempty(method)
        method = 'joint_cubic';
    end

    % Time vector
    dt = C.traj_dt;
    t = (0:dt:Tf)';
    K = length(t);

    % Initialize trajectory
    traj.t = t;
    violations = struct('limit', false, 'continuity', false, 'jerk', false, 'clearance', false);
    valid = true;

    % Determine if input is task-space or joint-space
    if length(start_pose) == 4 && length(goal_pose) == 4
        % Task-space: need IK to get joint configurations
        mode = 'task';
    else
        mode = 'joint';
    end

    % Generate trajectory based on method
    switch lower(method)
        case 'joint_cubic'
            if strcmp(mode, 'task')
                % Convert task-space to joint-space via IK
                q0_guess = zeros(C.n, 1);  % Initial guess at home
                [q_start, status_start, ~, ~, ~] = ik_dls(start_pose, q0_guess, C);
                if ~strcmp(status_start, 'success')
                    error('plan_trajectory:IKFailed', 'IK failed for start pose: %s', status_start);
                end

                [q_goal, status_goal, ~, ~, ~] = ik_dls(goal_pose, q_start, C);
                if ~strcmp(status_goal, 'success')
                    error('plan_trajectory:IKFailed', 'IK failed for goal pose: %s', status_goal);
                end
            else
                q_start = start_pose;
                q_goal = goal_pose;
            end

            % Cubic polynomial interpolation (zero velocity at endpoints)
            traj = plan_joint_cubic(q_start, q_goal, t, C);

        case 'task_lspb'
            if strcmp(mode, 'joint')
                error('plan_trajectory:InvalidMethod', 'task_lspb requires task-space poses');
            end

            traj = plan_task_lspb(start_pose, goal_pose, t, C);

        otherwise
            error('plan_trajectory:UnknownMethod', 'Unknown method: %s', method);
    end

    % Validate trajectory constraints
    [valid, violations] = validate_trajectory(traj, C);

    % Compute clearance along trajectory
    traj.clearance = compute_clearance(traj.q, C);

    if traj.clearance < C.clearance
        violations.clearance = true;
        valid = false;
    end

end


function traj = plan_joint_cubic(q_start, q_goal, t, C)
% Cubic polynomial in joint space with zero velocity at endpoints

    Tf = t(end);
    K = length(t);
    n = C.n;

    % Cubic polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    % Boundary conditions: q(0)=q0, q(Tf)=qf, qd(0)=0, qd(Tf)=0
    % Solution:
    %   a0 = q0
    %   a1 = 0
    %   a2 = 3*(qf-q0)/Tf^2
    %   a3 = -2*(qf-q0)/Tf^3

    traj.q = zeros(K, n);
    traj.qd = zeros(K, n);
    traj.qdd = zeros(K, n);
    traj.pose = zeros(4, 4, K);

    for i = 1:n
        a0 = q_start(i);
        a1 = 0;
        a2 = 3 * (q_goal(i) - q_start(i)) / Tf^2;
        a3 = -2 * (q_goal(i) - q_start(i)) / Tf^3;

        traj.q(:, i) = a0 + a1*t + a2*t.^2 + a3*t.^3;
        traj.qd(:, i) = a1 + 2*a2*t + 3*a3*t.^2;
        traj.qdd(:, i) = 2*a2 + 6*a3*t;
    end

    % Compute forward kinematics along trajectory
    for k = 1:K
        [T, ~, ~] = fk(traj.q(k, :)', C);
        traj.pose(:, :, k) = T;
    end

    traj.t = t;

end


function traj = plan_task_lspb(start_pose, goal_pose, t, C)
% Linear Segment with Parabolic Blends (LSPB) in task space

    Tf = t(end);
    K = length(t);
    n = C.n;

    % Extract positions and yaw
    pos_start = start_pose(1:3);
    yaw_start = start_pose(4);
    pos_goal = goal_pose(1:3);
    yaw_goal = goal_pose(4);

    % Total distance
    dist = norm(pos_goal - pos_start);
    dyaw = wrapToPi(yaw_goal - yaw_start);

    % LSPB parameters (symmetric blend)
    tb = Tf / 4;  % Blend time (25% of total time)
    V = dist / (Tf - tb);  % Cruise velocity

    % Position trajectory
    pos_traj = zeros(K, 3);
    yaw_traj = zeros(K, 1);

    for k = 1:K
        tk = t(k);

        if tk <= tb
            % Acceleration phase
            s = 0.5 * (V/tb) * tk^2;
        elseif tk <= (Tf - tb)
            % Constant velocity phase
            s = V * (tk - tb/2);
        else
            % Deceleration phase
            s = dist - 0.5 * (V/tb) * (Tf - tk)^2;
        end

        % Normalize and interpolate
        if dist > 1e-6
            lambda = s / dist;
        else
            lambda = tk / Tf;
        end

        lambda = max(0, min(1, lambda));  % Clamp to [0,1]

        pos_traj(k, :) = pos_start' + lambda * (pos_goal - pos_start)';
        yaw_traj(k) = yaw_start + lambda * dyaw;
    end

    % Compute joint trajectory via IK
    traj.q = zeros(K, n);
    traj.qd = zeros(K, n);
    traj.qdd = zeros(K, n);
    traj.pose = zeros(4, 4, K);

    q_prev = zeros(n, 1);  % Initial guess

    for k = 1:K
        target = [pos_traj(k, :)'; yaw_traj(k)];
        [q_sol, status, ~, ~, ~] = ik_dls(target, q_prev, C);

        if ~strcmp(status, 'success')
            warning('plan_trajectory:IKFailed', 'IK failed at step %d/%d: %s', k, K, status);
        end

        traj.q(k, :) = q_sol';
        q_prev = q_sol;

        [T, ~, ~] = fk(q_sol, C);
        traj.pose(:, :, k) = T;
    end

    % Numerical differentiation for velocities and accelerations
    dt = t(2) - t(1);
    traj.qd(2:end, :) = diff(traj.q) / dt;
    traj.qd(1, :) = traj.qd(2, :);  % Extrapolate first point

    traj.qdd(2:end, :) = diff(traj.qd) / dt;
    traj.qdd(1, :) = traj.qdd(2, :);  % Extrapolate first point

    traj.t = t;

end


function [valid, violations] = validate_trajectory(traj, C)
% Validate trajectory against constraints

    valid = true;
    violations = struct('limit', false, 'continuity', false, 'jerk', false, 'clearance', false);

    % 1. Joint limit violations
    for i = 1:C.n
        if any(traj.q(:, i) < C.qmin(i)) || any(traj.q(:, i) > C.qmax(i))
            violations.limit = true;
            valid = false;
            warning('validate_trajectory:LimitViolation', 'Joint %d exceeds limits', i);
        end
    end

    % 2. Velocity continuity (check for jumps)
    dt = traj.t(2) - traj.t(1);
    for i = 1:C.n
        dqd = diff(traj.qd(:, i));
        if any(abs(dqd) > 10 * C.vmax * dt)  % Large jump in velocity
            violations.continuity = true;
            valid = false;
            warning('validate_trajectory:Discontinuity', 'Velocity discontinuity in joint %d', i);
        end
    end

    % 3. Jerk spikes
    for i = 1:C.n
        jerk = diff(traj.qdd(:, i)) / dt;
        median_jerk = median(abs(jerk));
        max_jerk = max(abs(jerk));

        if max_jerk > C.jerk_threshold_ratio * median_jerk && median_jerk > 1e-6
            violations.jerk = true;
            valid = false;
            warning('validate_trajectory:JerkSpike', 'Excessive jerk in joint %d: %.2e (%.2f× median)', ...
                    i, max_jerk, max_jerk/median_jerk);
        end
    end

end


function clearance = compute_clearance(q_traj, C)
% Compute minimum clearance along trajectory

    K = size(q_traj, 1);
    min_clearance = inf;

    for k = 1:K
        [T, ~, ~] = fk(q_traj(k, :)', C);
        z_ee = T(3, 4);

        % Clearance above tray frame
        clearance_k = z_ee - C.z_tray + C.clearance;
        min_clearance = min(min_clearance, clearance_k);
    end

    clearance = min_clearance;

end

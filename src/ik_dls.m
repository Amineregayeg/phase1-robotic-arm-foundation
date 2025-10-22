function [q_sol, status, iters, final_error, singularity_events] = ik_dls(T_target, q0, C)
% IK_DLS - Inverse kinematics using damped least squares
%
% Solves inverse kinematics for position + yaw using damped least squares
% (DLS) with adaptive damping near singularities.
%
% Inputs:
%   T_target - Target pose (4×4 homogeneous transform) OR
%              [x, y, z, yaw] (4×1) for position + yaw
%   q0       - Initial guess for joint angles (n×1) [rad]
%   C        - Configuration struct from config()
%
% Outputs:
%   q_sol     - Solution joint angles (n×1) [rad] (or q0 if failed)
%   status    - String: 'success', 'max_iters', 'unreachable', 'singularity'
%   iters     - Number of iterations performed
%   final_error - Final position and orientation error [pos_err; yaw_err]
%   singularity_events - Number of times damping was increased
%
% Method: Iterative Newton-Raphson with damped least squares
%   Δq = (J^T * J + λ²I)^(-1) * J^T * Δx
%   λ adapts based on condition number
%
% Target: Position (x, y, z) + yaw (rotation about z-axis)
%   This gives 4 DOF constraint for 5-DOF arm, leaving null space
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    % Input validation
    if nargin < 3
        C = config();
    end

    if size(q0, 2) ~= 1
        q0 = q0(:);
    end

    % Parse target
    if numel(T_target) == 16  % Homogeneous transform
        pos_target = T_target(1:3, 4);
        R_target = T_target(1:3, 1:3);
        % Extract yaw from rotation matrix (assuming planar operation)
        yaw_target = atan2(R_target(2,1), R_target(1,1));
    elseif numel(T_target) == 4  % [x, y, z, yaw]
        pos_target = T_target(1:3);
        yaw_target = T_target(4);
    else
        error('ik_dls:InvalidTarget', 'Target must be 4×4 transform or [x,y,z,yaw]');
    end

    % Initialize
    q = enforce_limits(q0, C);
    lambda = C.lambda_init;
    singularity_events = 0;
    status = 'max_iters';

    % Iteration loop
    for iters = 1:C.max_iters
        % Forward kinematics
        [T_current, ~, valid] = fk(q, C);

        if ~valid
            status = 'singularity';
            break;
        end

        % Current pose
        pos_current = T_current(1:3, 4);
        R_current = T_current(1:3, 1:3);
        yaw_current = atan2(R_current(2,1), R_current(1,1));

        % Error vector (4×1: position + yaw)
        pos_error = pos_target - pos_current;
        yaw_error = wrapToPi(yaw_target - yaw_current);
        error_vec = [pos_error; yaw_error];

        % Check convergence
        pos_err_norm = norm(pos_error);
        yaw_err_abs = abs(yaw_error);

        if pos_err_norm < C.tol_pos && yaw_err_abs < C.tol_yaw
            status = 'success';
            final_error = [pos_err_norm; yaw_err_abs];
            q_sol = q;
            return;
        end

        % Compute Jacobian
        [J_full, manip, cond_J] = jacobian_geometric(q, C);

        % Use only position + yaw rows (rows 1:3 for position, row 6 for z-rotation)
        J = [J_full(1:3, :); J_full(6, :)];  % 4×n Jacobian

        % Adaptive damping based on condition number
        if cond_J > C.cond_threshold
            lambda = min(lambda * 2, C.lambda_max);
            singularity_events = singularity_events + 1;
        else
            lambda = max(lambda / 1.5, C.lambda_init);
        end

        % Damped least squares step
        % Δq = (J^T*J + λ²I)^(-1) * J^T * Δx
        n = C.n;
        JTJ = J' * J;
        damping_matrix = lambda^2 * eye(n);

        dq = (JTJ + damping_matrix) \ (J' * error_vec);

        % Adaptive step size near singularities
        if cond_J > C.cond_threshold
            step_size = 0.5;  % Slow down near singularities
        else
            step_size = 1.0;
        end

        % Update joint angles
        q = q + step_size * dq;

        % Enforce joint limits
        q = enforce_limits(q, C);
    end

    % If we exit loop without convergence
    final_error = [norm(pos_target - T_current(1:3, 4)); abs(wrapToPi(yaw_target - yaw_current))];

    % Check if target is likely unreachable
    if final_error(1) > 0.05  % More than 5cm error
        status = 'unreachable';
    end

    q_sol = q;

end

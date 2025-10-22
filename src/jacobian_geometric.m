function [J, manip, cond_J] = jacobian_geometric(q, C)
% JACOBIAN_GEOMETRIC - Compute geometric Jacobian using screw axes
%
% Computes the 6×n geometric Jacobian relating joint velocities to
% end-effector twist (linear and angular velocity).
%
% Inputs:
%   q - Joint angles (n×1) [rad]
%   C - Configuration struct from config()
%
% Outputs:
%   J      - Geometric Jacobian (6×n) [top 3 rows: linear, bottom 3: angular]
%   manip  - Manipulability index w = sqrt(det(J*J'))
%   cond_J - Condition number of J
%
% Method: Geometric approach using z-axes and position vectors
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    % Input validation
    if nargin < 2
        C = config();
    end

    n = length(q);
    if n ~= C.n
        error('jacobian_geometric:InvalidInput', 'Joint vector must have %d elements', C.n);
    end

    % Compute forward kinematics to get all transforms
    [T0N, Ti, ~] = fk(q, C);

    % End-effector position
    p_n = T0N(1:3, 4);

    % Initialize Jacobian
    J = zeros(6, n);

    % For revolute joints, each column is:
    % J_i = [z_{i-1} × (p_n - p_{i-1}); z_{i-1}]
    % where z_{i-1} is the z-axis of frame i-1

    for i = 1:n
        if i == 1
            % Base frame (identity)
            z_prev = [0; 0; 1];
            p_prev = [0; 0; 0];
        else
            % Previous frame transform
            T_prev = Ti(:, :, i-1);
            z_prev = T_prev(1:3, 3);  % z-axis (3rd column of rotation)
            p_prev = T_prev(1:3, 4);  % position
        end

        % Linear velocity contribution: z × (p_n - p_prev)
        J(1:3, i) = cross(z_prev, p_n - p_prev);

        % Angular velocity contribution: z
        J(4:6, i) = z_prev;
    end

    % Compute manipulability
    % w = sqrt(det(J * J^T))
    JJT = J * J';
    det_JJT = det(JJT);

    if det_JJT < 0
        warning('jacobian_geometric:NegativeDet', 'Negative determinant in manipulability calculation');
        manip = 0;
    else
        manip = sqrt(det_JJT);
    end

    % Compute condition number
    % Use singular values for numerical stability
    sigma = svd(J);
    if sigma(end) < 1e-10
        cond_J = inf;
    else
        cond_J = sigma(1) / sigma(end);
    end

end

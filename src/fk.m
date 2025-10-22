function [T0N, Ti, valid] = fk(q, C)
% FK - Forward kinematics using standard DH convention
%
% Computes the forward kinematics transformation matrices from base to
% end-effector and all intermediate frames using standard DH parameters.
%
% Inputs:
%   q - Joint angles (n×1) [rad]
%   C - Configuration struct from config()
%
% Outputs:
%   T0N   - Homogeneous transformation from base to end-effector (4×4)
%   Ti    - Transformations to each frame (4×4×n)
%   valid - Boolean, true if all rotation matrices are orthonormal
%
% DH Standard Convention:
%   T_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    % Input validation
    if nargin < 2
        C = config();
    end

    n = length(q);
    if n ~= C.n
        error('fk:InvalidInput', 'Joint vector must have %d elements', C.n);
    end

    if size(q, 2) ~= 1
        q = q(:);  % Force column vector
    end

    % Initialize transformation matrices
    Ti = zeros(4, 4, n);
    T0N = eye(4);
    valid = true;

    % Build transformation for each joint
    for i = 1:n
        % Extract DH parameters
        theta_i = q(i) + C.theta0(i);
        d_i = C.d(i);
        a_i = C.a(i);
        alpha_i = C.alpha(i);

        % Standard DH transformation matrix
        % T_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
        ct = cos(theta_i);
        st = sin(theta_i);
        ca = cos(alpha_i);
        sa = sin(alpha_i);

        T_i = [
            ct,    -st*ca,   st*sa,    a_i*ct;
            st,     ct*ca,  -ct*sa,    a_i*st;
            0,      sa,      ca,       d_i;
            0,      0,       0,        1
        ];

        % Accumulate transformation
        T0N = T0N * T_i;
        Ti(:, :, i) = T0N;

        % Validate rotation matrix orthonormality
        R = T0N(1:3, 1:3);
        orthonorm_error = norm(R' * R - eye(3), 'fro');

        if orthonorm_error > 1e-6
            valid = false;
            warning('fk:Orthonormality', ...
                    'Frame %d: Rotation matrix orthonormality error = %.2e (threshold: 1e-6)', ...
                    i, orthonorm_error);
        end

        % Check for NaN or Inf
        if any(~isfinite(T0N(:)))
            valid = false;
            error('fk:NonFinite', 'Frame %d contains NaN or Inf values', i);
        end
    end

    % Final validation
    R_final = T0N(1:3, 1:3);
    final_orthonorm_error = norm(R_final' * R_final - eye(3), 'fro');

    if final_orthonorm_error > 1e-6
        valid = false;
    end

end

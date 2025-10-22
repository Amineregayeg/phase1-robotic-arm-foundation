function q_clamped = enforce_limits(q, C)
% ENFORCE_LIMITS - Clamp joint angles to physical limits
%
% Ensures all joint angles are within the specified joint limits.
%
% Inputs:
%   q - Joint angles (n×1) [rad]
%   C - Configuration struct from config()
%
% Outputs:
%   q_clamped - Joint angles after clamping (n×1) [rad]
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    % Input validation
    if nargin < 2
        C = config();
    end

    if size(q, 2) ~= 1
        q = q(:);  % Force column vector
    end

    % Clamp each joint
    q_clamped = q;
    for i = 1:length(q)
        if q(i) < C.qmin(i)
            q_clamped(i) = C.qmin(i);
        elseif q(i) > C.qmax(i)
            q_clamped(i) = C.qmax(i);
        end
    end

end

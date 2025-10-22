function [det_JJT, cond_J, manip, is_singular] = singularity_metrics(q, C)
% SINGULARITY_METRICS - Compute singularity and conditioning metrics
%
% Computes various metrics to assess proximity to singularities:
%   - det(J*J^T): Determinant of Gram matrix
%   - cond(J): Condition number of Jacobian
%   - manipulability: sqrt(det(J*J^T))
%   - Singularity flag based on thresholds
%
% Inputs:
%   q - Joint angles (n×1) [rad]
%   C - Configuration struct from config()
%
% Outputs:
%   det_JJT    - Determinant of J*J^T
%   cond_J     - Condition number of J
%   manip      - Manipulability index
%   is_singular - Boolean, true if near singular (cond > threshold)
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    % Input validation
    if nargin < 2
        C = config();
    end

    % Compute Jacobian and metrics
    [J, manip, cond_J] = jacobian_geometric(q, C);

    % Compute det(J*J^T)
    JJT = J * J';
    det_JJT = det(JJT);

    % Check singularity based on condition number
    is_singular = (cond_J > C.cond_threshold) || (manip < C.manip_threshold);

end

function utils_plotting(action, varargin)
% UTILS_PLOTTING - Standardized plotting utilities for consistent figures
%
% Provides consistent styling for all Phase-1 plots: fonts, DPI, colors, etc.
%
% Usage:
%   utils_plotting('setup') - Setup default plot settings
%   utils_plotting('save', filename) - Save current figure with standard settings
%   utils_plotting('style_3d', ax) - Apply 3D plot styling
%   utils_plotting('style_2d', ax) - Apply 2D plot styling
%
% Author: Phase-1 Foundation Team
% Date: 2025-10-22

    C = config();

    switch lower(action)
        case 'setup'
            % Set default figure properties
            set(0, 'DefaultFigureColor', 'w');
            set(0, 'DefaultAxesFontSize', C.font_size);
            set(0, 'DefaultAxesFontName', 'Helvetica');
            set(0, 'DefaultTextFontSize', C.font_size);
            set(0, 'DefaultLineLineWidth', 1.5);
            set(0, 'DefaultAxesBox', 'on');
            set(0, 'DefaultAxesLineWidth', 1.0);

        case 'save'
            if nargin < 2
                error('utils_plotting:MissingFilename', 'Filename required for save action');
            end
            filename = varargin{1};

            % Ensure figs directory exists
            [filepath, ~, ~] = fileparts(filename);
            if ~isempty(filepath) && ~exist(filepath, 'dir')
                mkdir(filepath);
            end

            % Save with high resolution
            fig = gcf;
            fig.PaperPositionMode = 'auto';
            fig.PaperUnits = 'inches';
            pos = fig.PaperPosition;
            fig.PaperSize = [pos(3), pos(4)];

            print(fig, filename, sprintf('-d%s', C.fig_format), sprintf('-r%d', C.fig_dpi));

        case 'style_3d'
            if nargin < 2
                ax = gca;
            else
                ax = varargin{1};
            end

            % Apply 3D styling
            grid(ax, 'on');
            box(ax, 'on');
            xlabel(ax, 'X [m]', 'FontSize', C.font_size);
            ylabel(ax, 'Y [m]', 'FontSize', C.font_size);
            zlabel(ax, 'Z [m]', 'FontSize', C.font_size);
            axis(ax, 'equal');
            view(ax, 3);

        case 'style_2d'
            if nargin < 2
                ax = gca;
            else
                ax = varargin{1};
            end

            % Apply 2D styling
            grid(ax, 'on');
            box(ax, 'on');
            xlabel(ax, 'X [m]', 'FontSize', C.font_size);
            ylabel(ax, 'Y [m]', 'FontSize', C.font_size);
            axis(ax, 'equal');

        case 'colormap_jet'
            colormap(jet);
            colorbar;

        otherwise
            error('utils_plotting:UnknownAction', 'Unknown action: %s', action);
    end

end

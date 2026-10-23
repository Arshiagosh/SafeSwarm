% startup.m — SafeSwarm project initialisation
%
% Run this once after opening MATLAB in the project root (or let MATLAB run
% it automatically if the project is your startup folder). It puts every
% SafeSwarm class on the path and applies a clean, consistent plotting style
% (LaTeX labels, grids, readable fonts) so analysis figures look publication
% ready out of the box.

%% Path — make all SafeSwarm classes available from anywhere
here = fileparts(mfilename('fullpath'));
addpath(genpath(here));

%% Plot style (clean, LaTeX, analysis-friendly)
set(groot, 'defaulttextinterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex');
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'best');
set(groot, 'defaultFigureColor',                'w');
set(groot, 'defaultLineLineWidth',              1.8);
set(groot, 'defaultAxesFontSize',               14);
set(groot, 'DefaultAxesXGrid',                  'on');
set(groot, 'DefaultAxesYGrid',                  'on');
set(groot, 'DefaultAxesGridLineStyle',          '--');
set(groot, 'DefaultAxesGridAlpha',              0.5);

% Dock figures only when a desktop is available (avoids warnings in -batch).
if usejava('desktop')
    set(groot, 'defaultFigureWindowStyle', 'docked');
end

fprintf('SafeSwarm ready — %d classes on path. Try: run(''scenarios/scenario_flocking.m'')\n', ...
        numel(what(fullfile(here, 'core')).m));

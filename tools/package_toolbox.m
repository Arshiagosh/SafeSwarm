function file = package_toolbox(version)
    % package_toolbox(version) — build release/SafeSwarm.mltbx
    %
    %   package_toolbox()        version '1.0.0'
    %   package_toolbox('1.1.0')
    %
    % The toolbox contains the library folders only (no experiments output,
    % tests or figures). Uses matlab.addons.toolbox.ToolboxOptions (R2023a+).

    if nargin < 1, version = '1.0.0'; end
    root = fileparts(fileparts(mfilename('fullpath')));
    out  = fullfile(root, 'release');
    if ~exist(out, 'dir'), mkdir(out); end

    opts = matlab.addons.toolbox.ToolboxOptions(root, 'safeswarm-arshiagosh');
    opts.ToolboxName    = 'SafeSwarm';
    opts.ToolboxVersion = version;
    opts.AuthorName     = 'Arshia Goshtasbi';
    opts.AuthorEmail    = 'arshia.goshtasbi@gmail.com';
    opts.Summary        = 'Safe multi-robot control: CBF-QP safety filters, uncertainty, MPC and a ROS 2 bridge.';
    opts.Description    = ['Bounded-input robot models, centralised and decentralised CBF-QP ' ...
                           'safety filters, sampled-data checks, sensor noise, lossy communication ' ...
                           'and faults, distributed NMPC, a GP disturbance model and ROS 2 nodes.'];
    opts.MinimumMatlabRelease = 'R2023a';

    keep = {'core', 'dynamics', 'behaviours', 'planners', 'safety', 'uncertainty', ...
            'mpc', 'ros2', 'tools', 'visualization', 'scenarios', 'docs'};
    files = {};
    for k = 1:numel(keep)
        d = dir(fullfile(root, keep{k}, '**', '*.*'));
        d = d(~[d.isdir]);
        files = [files, fullfile({d.folder}, {d.name})]; %#ok<AGROW>
    end
    files = [files, {fullfile(root, 'startup.m'), fullfile(root, 'README.md'), ...
                     fullfile(root, 'LICENSE')}];
    opts.ToolboxFiles = files;
    opts.ToolboxMatlabPath = fullfile(root, keep(~strcmp(keep, 'docs')));
    opts.OutputFile = fullfile(out, 'SafeSwarm.mltbx');

    matlab.addons.toolbox.packageToolbox(opts);
    file = opts.OutputFile;
    fprintf('Built %s\n', file);
end

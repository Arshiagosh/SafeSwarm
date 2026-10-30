function save_results(key, value)
    % save_results(key, value) — store value under key in results.json at the repo root
    %   Other keys already in the file are kept, so each experiment updates its own part.
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    file = fullfile(root, 'results.json');
    % A lock folder keeps experiments that run in parallel from overwriting each other.
    lock = [file '.lock'];
    t0 = tic;
    while ~mkdir_new(lock)
        if toc(t0) > 120, error('save_results:Lock', 'Could not lock %s', file); end
        pause(0.2);
    end
    cleanup = onCleanup(@() rmdir(lock));
    data = struct();
    if isfile(file)
        data = jsondecode(fileread(file));
    end
    data.(key) = value;
    data.generated_with = sprintf('MATLAB %s', version('-release'));
    fid = fopen(file, 'w');
    fprintf(fid, '%s\n', jsonencode(data, 'PrettyPrint', true));
    fclose(fid);
end

function ok = mkdir_new(d)
    % True only for the caller that actually created the folder.
    if isfolder(d), ok = false; return; end
    [ok, ~, id] = mkdir(d);
    ok = ok && ~strcmp(id, 'MATLAB:MKDIR:DirectoryExists');
end

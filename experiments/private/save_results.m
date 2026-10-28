function save_results(key, value)
    % save_results(key, value) — store value under key in results.json at the repo root
    %   Other keys already in the file are kept, so each experiment updates its own part.
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    file = fullfile(root, 'results.json');
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

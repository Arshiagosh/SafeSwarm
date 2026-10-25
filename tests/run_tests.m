function results = run_tests()
    % run_tests — run every SafeSwarm unit test and print a summary.
    %
    % Works from any folder:  run('tests/run_tests.m')  or  run_tests
    % Errors if a test fails, so it can gate CI.

    here = fileparts(mfilename('fullpath'));
    addpath(genpath(fileparts(here)));
    results = runtests(here);

    % runtests skips a test file it cannot parse with only a warning; fail instead.
    files   = dir(fullfile(here, '*Test.m'));
    classes = unique(extractBefore(string({results.Name}), '/'));
    missing = setdiff(erase(string({files.name}), '.m'), classes);
    if ~isempty(missing)
        error('run_tests:Skipped', 'These test files did not run: %s', strjoin(missing, ', '));
    end
    disp(table(results));
    fprintf('%d passed, %d failed, %d incomplete\n', ...
            sum([results.Passed]), sum([results.Failed]), sum([results.Incomplete]));
    assertSuccess(results);
end

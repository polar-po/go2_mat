function results = run_tests()
%RUN_TESTS Run the headless project tests from the Go2 root.

projectRoot = fileparts(mfilename('fullpath'));
results = runtests(fullfile(projectRoot, 'tests'));
assertSuccess(results);
end
